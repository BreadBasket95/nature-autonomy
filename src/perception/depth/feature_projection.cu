//
// CUDA-backed sparse depth projection from VIO landmarks.
//
#include "nature/perception/depth/feature_projector.h"

#include <algorithm>
#include <cmath>
#include <vector>

#ifdef NATURE_HAS_CUDA
#include <cuda_runtime.h>
#endif

namespace nature {
namespace perception {
namespace depth {

namespace {

#ifdef NATURE_HAS_CUDA
#define NATURE_HOST_DEVICE __host__ __device__
#else
#define NATURE_HOST_DEVICE
#endif

struct Float3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  NATURE_HOST_DEVICE Float3() = default;
  NATURE_HOST_DEVICE Float3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct Mat4f {
  float m[16] = {0.0f};
  NATURE_HOST_DEVICE Float3 transform_point(const Float3 &p) const {
    const float x = m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3];
    const float y = m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7];
    const float z = m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11];
    return Float3(x, y, z);
  }
};

Mat4f ToMat4f(const Eigen::Matrix4f &mat) {
  Mat4f out;
  // Store row-major for direct multiplication in kernels.
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out.m[r * 4 + c] = mat(r, c);
    }
  }
  return out;
}

constexpr float kDepthSentinel = 1.0e6f;

#ifdef NATURE_HAS_CUDA
__device__ float atomic_min_float(float *addr, float value) {
  int *address_as_i = reinterpret_cast<int *>(addr);
  int old = *address_as_i;
  while (__int_as_float(old) > value) {
    const int assumed = old;
    old = atomicCAS(address_as_i, assumed, __float_as_int(value));
    if (assumed == old) {
      break;
    }
  }
  return __int_as_float(old);
}

__global__ void clear_depth_kernel(float *buffer, int count, float sentinel) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < count) {
    buffer[idx] = sentinel;
  }
}

__global__ void normalize_depth_kernel(float *buffer, int count, float sentinel) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < count) {
    if (buffer[idx] >= sentinel * 0.5f) {
      buffer[idx] = 0.0f;
    }
  }
}

__global__ void project_landmarks_kernel(const Float3 *landmarks_w,
                                         int count,
                                         Mat4f T_cw,
                                         CameraIntrinsics K,
                                         float *depth_out) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= count) {
    return;
  }
  const Float3 p_w = landmarks_w[idx];
  const Float3 p_c = T_cw.transform_point(p_w);
  if (p_c.z <= 0.0f) {
    return;
  }
  const float u = K.fx * p_c.x / p_c.z + K.cx;
  const float v = K.fy * p_c.y / p_c.z + K.cy;
  const int ui = static_cast<int>(u + 0.5f);
  const int vi = static_cast<int>(v + 0.5f);
  if (ui < 0 || vi < 0 || ui >= K.width || vi >= K.height) {
    return;
  }
  const int pixel_index = vi * K.width + ui;
  atomic_min_float(&depth_out[pixel_index], p_c.z);
}
#endif

void generate_sparse_depth_cpu(const vio::VIOUpdate &vio,
                               const Eigen::Matrix4f &T_cw,
                               const CameraIntrinsics &K,
                               float *output_buffer) {
  const int pixel_count = K.width * K.height;
  std::vector<float> local_depth(pixel_count, kDepthSentinel);
  Mat4f T_cw_host = ToMat4f(T_cw);
  for (const auto &pt : vio.landmarks_w) {
    const Float3 p_w(pt.x(), pt.y(), pt.z());
    const Float3 p_c = T_cw_host.transform_point(p_w);
    if (p_c.z <= 0.0f) {
      continue;
    }
    const float u = K.fx * p_c.x / p_c.z + K.cx;
    const float v = K.fy * p_c.y / p_c.z + K.cy;
    const int ui = static_cast<int>(u + 0.5f);
    const int vi = static_cast<int>(v + 0.5f);
    if (ui < 0 || vi < 0 || ui >= K.width || vi >= K.height) {
      continue;
    }
    const int pixel_index = vi * K.width + ui;
    local_depth[pixel_index] = std::min(local_depth[pixel_index], p_c.z);
  }
  for (int i = 0; i < pixel_count; ++i) {
    output_buffer[i] = (local_depth[i] >= kDepthSentinel * 0.5f) ? 0.0f : local_depth[i];
  }
}

} // namespace

// ============================================================================
// FeatureProjector implementation with persistent GPU buffers
// ============================================================================

struct FeatureProjector::Impl {
#ifdef NATURE_HAS_CUDA
  Float3 *device_landmarks = nullptr;
  size_t device_landmarks_capacity = 0;
#endif
  std::vector<Float3> host_landmarks;
};

FeatureProjector::FeatureProjector() : impl_(new Impl()) {}

FeatureProjector::~FeatureProjector() {
#ifdef NATURE_HAS_CUDA
  if (impl_ && impl_->device_landmarks) {
    cudaFree(impl_->device_landmarks);
    impl_->device_landmarks = nullptr;
  }
#endif
}

void FeatureProjector::generate_sparse_depth_map(const vio::VIOUpdate &vio,
                                                 const Eigen::Matrix4f &T_bc,
                                                 const CameraIntrinsics &K,
                                                 float *gpu_output_buffer,
                                                 cudaStream_t stream) {
  if (!gpu_output_buffer || K.width <= 0 || K.height <= 0) {
    return;
  }

  const int pixel_count = K.width * K.height;
  const Eigen::Matrix4f T_cw = T_bc * vio.T_wb;

#ifdef NATURE_HAS_CUDA
  const int block = 256;
  const int grid = (pixel_count + block - 1) / block;
  clear_depth_kernel<<<grid, block, 0, stream>>>(gpu_output_buffer, pixel_count, kDepthSentinel);

  if (!vio.landmarks_w.empty()) {
    // Prepare host landmarks
    impl_->host_landmarks.clear();
    impl_->host_landmarks.reserve(vio.landmarks_w.size());
    for (const auto &pt : vio.landmarks_w) {
      impl_->host_landmarks.emplace_back(pt.x(), pt.y(), pt.z());
    }

    const size_t required_bytes = impl_->host_landmarks.size() * sizeof(Float3);

    // Resize device buffer only if needed (with some growth factor to reduce reallocations)
    if (required_bytes > impl_->device_landmarks_capacity) {
      if (impl_->device_landmarks) {
        cudaFree(impl_->device_landmarks);
      }
      // Allocate with 1.5x growth factor to reduce future reallocations
      const size_t new_capacity = static_cast<size_t>(required_bytes * 1.5);
      cudaMalloc(&impl_->device_landmarks, new_capacity);
      impl_->device_landmarks_capacity = new_capacity;
    }

    cudaMemcpyAsync(impl_->device_landmarks, impl_->host_landmarks.data(),
                    required_bytes, cudaMemcpyHostToDevice, stream);

    const int grid_landmarks = (static_cast<int>(impl_->host_landmarks.size()) + block - 1) / block;
    const Mat4f T_cw_device = ToMat4f(T_cw);
    project_landmarks_kernel<<<grid_landmarks, block, 0, stream>>>(
        impl_->device_landmarks, static_cast<int>(impl_->host_landmarks.size()),
        T_cw_device, K, gpu_output_buffer);
  }

  normalize_depth_kernel<<<grid, block, 0, stream>>>(gpu_output_buffer, pixel_count, kDepthSentinel);
#else
  generate_sparse_depth_cpu(vio, T_cw, K, gpu_output_buffer);
#endif
}

// ============================================================================
// Legacy stateless function (allocates per call)
// ============================================================================

void generate_sparse_depth_map(const vio::VIOUpdate &vio,
                               const Eigen::Matrix4f &T_bc,
                               const CameraIntrinsics &K,
                               float *gpu_output_buffer,
                               cudaStream_t stream) {
  if (!gpu_output_buffer || K.width <= 0 || K.height <= 0) {
    return;
  }

  const int pixel_count = K.width * K.height;
  const Eigen::Matrix4f T_cw = T_bc * vio.T_wb;

#ifdef NATURE_HAS_CUDA
  const int block = 256;
  const int grid = (pixel_count + block - 1) / block;
  clear_depth_kernel<<<grid, block, 0, stream>>>(gpu_output_buffer, pixel_count, kDepthSentinel);

  if (!vio.landmarks_w.empty()) {
    std::vector<Float3> host_landmarks;
    host_landmarks.reserve(vio.landmarks_w.size());
    for (const auto &pt : vio.landmarks_w) {
      host_landmarks.emplace_back(pt.x(), pt.y(), pt.z());
    }

    Float3 *device_landmarks = nullptr;
    const size_t bytes = host_landmarks.size() * sizeof(Float3);
    cudaMalloc(&device_landmarks, bytes);
    cudaMemcpyAsync(device_landmarks, host_landmarks.data(), bytes, cudaMemcpyHostToDevice, stream);

    const int grid_landmarks = (static_cast<int>(host_landmarks.size()) + block - 1) / block;
    const Mat4f T_cw_device = ToMat4f(T_cw);
    project_landmarks_kernel<<<grid_landmarks, block, 0, stream>>>(
        device_landmarks, static_cast<int>(host_landmarks.size()), T_cw_device, K, gpu_output_buffer);

    normalize_depth_kernel<<<grid, block, 0, stream>>>(gpu_output_buffer, pixel_count, kDepthSentinel);
    cudaStreamSynchronize(stream);
    cudaFree(device_landmarks);
  } else {
    normalize_depth_kernel<<<grid, block, 0, stream>>>(gpu_output_buffer, pixel_count, kDepthSentinel);
  }
#else
  generate_sparse_depth_cpu(vio, T_cw, K, gpu_output_buffer);
#endif
}

} // namespace depth
} // namespace perception
} // namespace nature
