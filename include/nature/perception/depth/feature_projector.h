//
// Sparse depth projection utilities for VIO landmark projection.
//
#ifndef NATURE_FEATURE_PROJECTOR_H
#define NATURE_FEATURE_PROJECTOR_H

#include <Eigen/Core>

#include "nature/perception/vio/vio_estimator.h"

// Provide a fallback stream type when CUDA headers are unavailable.
#ifdef NATURE_HAS_CUDA
#include <cuda_runtime.h>
#else
typedef void *cudaStream_t;
#endif

namespace nature {
namespace perception {
namespace depth {

/**
 * @brief Camera intrinsics for projection operations.
 */
struct CameraIntrinsics {
  float fx = 0.0f;
  float fy = 0.0f;
  float cx = 0.0f;
  float cy = 0.0f;
  int width = 0;
  int height = 0;
};

/**
 * @brief Generate a sparse depth map from VIO landmarks.
 * @param vio VIO update containing world-frame landmarks and pose.
 * @param T_bc Body-to-camera transform.
 * @param K Camera intrinsics.
 * @param gpu_output_buffer Output depth buffer on the GPU (width*height floats).
 * @param stream CUDA stream to run kernels on (nullable for synchronous).
 * @details Projects landmarks into the camera frame, filters invalid points,
 *          and writes the nearest depth per pixel.
 */
void generate_sparse_depth_map(const vio::VIOUpdate &vio,
                               const Eigen::Matrix4f &T_bc,
                               const CameraIntrinsics &K,
                               float *gpu_output_buffer,
                               cudaStream_t stream);

} // namespace depth
} // namespace perception
} // namespace nature

#endif // NATURE_FEATURE_PROJECTOR_H
