//
// CUDA helpers for GPU-side occupancy grid mapping.
//
#include "nature/perception/mapping/occupancy_mapper.h"

#include <Eigen/Core>

#include <cuda_runtime.h>
#include <cmath>
#include <vector>

namespace nature {
namespace perception {
namespace mapping {

namespace {

struct Mat4f {
  float m[16] = {0.0f};
  __host__ __device__ float3 transform_point(const float3 &p) const {
    const float x = m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3];
    const float y = m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7];
    const float z = m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11];
    return make_float3(x, y, z);
  }
};

Mat4f to_mat4f(const Eigen::Matrix4f &mat) {
  Mat4f out;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out.m[r * 4 + c] = mat(r, c);
    }
  }
  return out;
}

__global__ void clear_grid_kernel(int *grid, int count, int value) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < count) {
    grid[idx] = value;
  }
}

__device__ void mark_cell(int *grid, int idx, int value) {
  // TODO: Switch to log-odds accumulation for probabilistic occupancy.
  atomicMax(&grid[idx], value);
}

__global__ void integrate_depth_kernel(const float *depth,
                                       const float *uncertainty,
                                       int width,
                                       int height,
                                       Mat4f T_wc,
                                       float fx, float fy, float cx, float cy,
                                       float resolution,
                                       float origin_x,
                                       float origin_y,
                                       int grid_w,
                                       int grid_h,
                                       float min_z,
                                       float max_z,
                                       float max_uncert,
                                       int cam_ix,
                                       int cam_iy,
                                       int free_val,
                                       int occ_val,
                                       int *grid) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  const int total = width * height;
  if (idx >= total) {
    return;
  }

  const float d = depth[idx];
  if (!isfinite(d) || d <= 0.0f) {
    return;
  }
  if (uncertainty && uncertainty[idx] > max_uncert) {
    return;
  }

  const int u = idx % width;
  const int v = idx / width;
  const float x = (static_cast<float>(u) - cx) * d / fx;
  const float y = (static_cast<float>(v) - cy) * d / fy;
  const float z = d;
  const float3 p_c = make_float3(x, y, z);
  const float3 p_w = T_wc.transform_point(p_c);
  if (p_w.z < min_z || p_w.z > max_z) {
    // TODO: Consider free-space updates even when the hit lies outside height band.
    return;
  }

  const int ix = static_cast<int>(floorf((p_w.x - origin_x) / resolution));
  const int iy = static_cast<int>(floorf((p_w.y - origin_y) / resolution));
  if (ix < 0 || iy < 0 || ix >= grid_w || iy >= grid_h) {
    return;
  }

  if (cam_ix >= 0 && cam_iy >= 0 && cam_ix < grid_w && cam_iy < grid_h) {
    // Bresenham ray to mark free space.
    int x0 = cam_ix;
    int y0 = cam_iy;
    int x1 = ix;
    int y1 = iy;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (x0 != x1 || y0 != y1) {
      const int grid_idx = y0 * grid_w + x0;
      mark_cell(grid, grid_idx, free_val);
      const int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }
  } else {
    // TODO: Handle rays when the camera origin is outside the grid.
  }

  const int hit_idx = iy * grid_w + ix;
  mark_cell(grid, hit_idx, occ_val);
}

} // namespace

void OccupancyMapper::allocate_gpu_buffers() {
  if (gpu_ready_) {
    return;
  }
  const int count = cells_x_ * cells_y_;
  cudaStreamCreate(&stream_);
  cudaMalloc(&grid_gpu_, static_cast<size_t>(count) * sizeof(int));
  gpu_ready_ = grid_gpu_ != nullptr;
  if (gpu_ready_) {
    const int block = 256;
    const int grid = (count + block - 1) / block;
    clear_grid_kernel<<<grid, block, 0, stream_>>>(grid_gpu_, count, static_cast<int>(unknown_value_));
    cudaStreamSynchronize(stream_);
  }
}

void OccupancyMapper::update_from_depth_gpu(const depth::DepthResult &depth,
                                            const Eigen::Matrix4f &T_wb,
                                            const Eigen::Matrix4f &T_bc,
                                            const depth::CameraIntrinsics &K) {
  if (!gpu_ready_ || !depth.dense_depth_gpu) {
    return;
  }

  const int width = depth.width > 0 ? depth.width : K.width;
  const int height = depth.height > 0 ? depth.height : K.height;
  if (width <= 0 || height <= 0) {
    return;
  }

  const Eigen::Matrix4f T_bw = T_wb.inverse();
  const Eigen::Matrix4f T_cb = T_bc.inverse();
  const Eigen::Matrix4f T_wc = T_bw * T_cb;

  const Eigen::Vector4f cam_origin = T_wc * Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
  const int cam_ix = static_cast<int>(std::floor((cam_origin.x() - origin_x_) / resolution_));
  const int cam_iy = static_cast<int>(std::floor((cam_origin.y() - origin_y_) / resolution_));

  const int count = cells_x_ * cells_y_;
  const int block = 256;
  const int grid = (count + block - 1) / block;
  clear_grid_kernel<<<grid, block, 0, stream_>>>(grid_gpu_, count, static_cast<int>(unknown_value_));

  const int pixels = width * height;
  const int pix_grid = (pixels + block - 1) / block;
  const Mat4f T_wc_device = to_mat4f(T_wc);

  integrate_depth_kernel<<<pix_grid, block, 0, stream_>>>(
      depth.dense_depth_gpu,
      depth.uncertainty_gpu,
      width,
      height,
      T_wc_device,
      K.fx, K.fy, K.cx, K.cy,
      resolution_,
      origin_x_,
      origin_y_,
      cells_x_,
      cells_y_,
      min_z_,
      max_z_,
      max_uncertainty_,
      cam_ix,
      cam_iy,
      0,
      100,
      grid_gpu_);

  cudaStreamSynchronize(stream_);
}

void OccupancyMapper::download_grid_locked() const {
  if (!gpu_ready_ || !grid_gpu_) {
    return;
  }
  const int count = cells_x_ * cells_y_;
  std::vector<int> host(count, static_cast<int>(unknown_value_));
  cudaMemcpy(host.data(), grid_gpu_, static_cast<size_t>(count) * sizeof(int), cudaMemcpyDeviceToHost);
  for (int i = 0; i < count; ++i) {
    int value = host[i];
    if (value > 100) {
      value = 100;
    }
    if (value < -1) {
      value = -1;
    }
    data_[static_cast<size_t>(i)] = static_cast<int8_t>(value);
  }
}

} // namespace mapping
} // namespace perception
} // namespace nature
