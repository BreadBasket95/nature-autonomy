//
// Occupancy grid mapper implementation for dense depth data.
//
#include "nature/perception/mapping/occupancy_mapper.h"

#include <algorithm>
#include <cmath>

#ifdef NATURE_HAS_CUDA
#include <cuda_runtime.h>
#endif

namespace nature {
namespace perception {
namespace mapping {

OccupancyMapper::OccupancyMapper(float grid_res, float width, float height,
                                 float origin_x, float origin_y)
    : resolution_(grid_res),
      width_(width),
      height_(height),
      origin_x_(origin_x),
      origin_y_(origin_y) {
  cells_x_ = static_cast<int>(std::ceil(width_ / resolution_));
  cells_y_ = static_cast<int>(std::ceil(height_ / resolution_));
  data_.assign(static_cast<size_t>(cells_x_ * cells_y_), unknown_value_);
#ifdef NATURE_HAS_CUDA
  allocate_gpu_buffers();
#endif
}

OccupancyMapper::~OccupancyMapper() {
#ifdef NATURE_HAS_CUDA
  if (grid_gpu_) {
    cudaFree(grid_gpu_);
    grid_gpu_ = nullptr;
  }
  if (stream_) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
#endif
}

void OccupancyMapper::set_height_band(float min_z, float max_z) {
  std::lock_guard<std::mutex> lock(mutex_);
  min_z_ = min_z;
  max_z_ = max_z;
}

void OccupancyMapper::set_uncertainty_threshold(float max_uncertainty) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_uncertainty_ = max_uncertainty;
}

void OccupancyMapper::set_use_gpu(bool enable) {
#ifdef NATURE_HAS_CUDA
  std::lock_guard<std::mutex> lock(mutex_);
  gpu_enabled_ = enable;
#else
  (void)enable;
#endif
}

void OccupancyMapper::update_from_depth(const depth::DepthResult &depth,
                                        const Eigen::Matrix4f &T_wb,
                                        const Eigen::Matrix4f &T_bc,
                                        const depth::CameraIntrinsics &K) {
  if (!depth.dense_depth_gpu) {
    return;
  }

#ifdef NATURE_HAS_CUDA
  if (gpu_enabled_ && gpu_ready_) {
    update_from_depth_gpu(depth, T_wb, T_bc, K);
    return;
  }
#endif

  update_from_depth_cpu(depth, T_wb, T_bc, K);
}

void OccupancyMapper::update_from_depth_cpu(const depth::DepthResult &depth,
                                            const Eigen::Matrix4f &T_wb,
                                            const Eigen::Matrix4f &T_bc,
                                            const depth::CameraIntrinsics &K) {
  const int width = depth.width > 0 ? depth.width : K.width;
  const int height = depth.height > 0 ? depth.height : K.height;
  if (width <= 0 || height <= 0) {
    return;
  }

  const size_t pixel_count = static_cast<size_t>(width * height);
  std::vector<float> depth_cpu(pixel_count, 0.0f);
  std::vector<float> uncertainty_cpu(pixel_count, 0.0f);

#ifdef NATURE_HAS_CUDA
  cudaMemcpy(depth_cpu.data(), depth.dense_depth_gpu,
             pixel_count * sizeof(float), cudaMemcpyDeviceToHost);
  if (depth.uncertainty_gpu) {
    cudaMemcpy(uncertainty_cpu.data(), depth.uncertainty_gpu,
               pixel_count * sizeof(float), cudaMemcpyDeviceToHost);
  }
#else
  std::copy_n(depth.dense_depth_gpu, pixel_count, depth_cpu.begin());
  if (depth.uncertainty_gpu) {
    std::copy_n(depth.uncertainty_gpu, pixel_count, uncertainty_cpu.begin());
  }
#endif

  const Eigen::Matrix4f T_bw = T_wb.inverse();
  const Eigen::Matrix4f T_cb = T_bc.inverse();
  const Eigen::Matrix4f T_wc = T_bw * T_cb;

  std::lock_guard<std::mutex> lock(mutex_);
  clear_grid_locked();

  constexpr int8_t kOccupiedValue = 100;
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const size_t idx = static_cast<size_t>(v * width + u);
      const float depth_m = depth_cpu[idx];
      if (!std::isfinite(depth_m) || depth_m <= 0.0f) {
        continue;
      }
      if (uncertainty_cpu[idx] > max_uncertainty_) {
        continue;
      }
      const float x = (static_cast<float>(u) - K.cx) * depth_m / K.fx;
      const float y = (static_cast<float>(v) - K.cy) * depth_m / K.fy;
      const float z = depth_m;

      const Eigen::Vector4f p_c(x, y, z, 1.0f);
      const Eigen::Vector4f p_w = T_wc * p_c;
      const float wz = p_w.z();
      if (wz < min_z_ || wz > max_z_) {
        continue;
      }

      const int ix = static_cast<int>(std::floor((p_w.x() - origin_x_) / resolution_));
      const int iy = static_cast<int>(std::floor((p_w.y() - origin_y_) / resolution_));
      if (ix < 0 || iy < 0 || ix >= cells_x_ || iy >= cells_y_) {
        continue;
      }

      // TODO: Add free-space raycasting for CPU fallback.
      data_[index_from_xy(ix, iy)] = kOccupiedValue;
    }
  }
}

nature::msg::OccupancyGrid OccupancyMapper::to_msg(bool row_major) const {
  std::lock_guard<std::mutex> lock(mutex_);
#ifdef NATURE_HAS_CUDA
  if (gpu_enabled_ && gpu_ready_) {
    download_grid_locked();
  }
#endif
  nature::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = resolution_;
  grid.info.width = static_cast<uint32_t>(cells_x_);
  grid.info.height = static_cast<uint32_t>(cells_y_);
  grid.info.origin.position.x = origin_x_;
  grid.info.origin.position.y = origin_y_;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(data_.begin(), data_.end());

  if (!row_major) {
    std::vector<int8_t> column_major;
    column_major.reserve(grid.data.size());
    for (int x = 0; x < cells_x_; ++x) {
      for (int y = 0; y < cells_y_; ++y) {
        column_major.push_back(data_[index_from_xy(x, y)]);
      }
    }
    grid.data.swap(column_major);
  }

  return grid;
}

void OccupancyMapper::clear_grid_locked() {
  // TODO: Allow configuring unknown/free values to match downstream planners.
  std::fill(data_.begin(), data_.end(), unknown_value_);
}

int OccupancyMapper::index_from_xy(int x, int y) const {
  return y * cells_x_ + x;
}

} // namespace mapping
} // namespace perception
} // namespace nature
