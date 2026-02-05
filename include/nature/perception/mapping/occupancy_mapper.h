//
// Occupancy grid mapper for dense depth outputs.
//
#ifndef NATURE_OCCUPANCY_MAPPER_H
#define NATURE_OCCUPANCY_MAPPER_H

#include <Eigen/Core>

#include <mutex>
#include <vector>

#include "nature/messaging/placeholder_messages.h"
#include "nature/perception/depth/depth_net.h"
#include "nature/perception/depth/feature_projector.h"

// Provide a fallback stream type when CUDA headers are unavailable.
#ifdef NATURE_HAS_CUDA
#include <cuda_runtime.h>
#else
typedef void *cudaStream_t;
#endif

namespace nature {
namespace perception {
namespace mapping {

/**
 * @brief Map dense depth into a 2D occupancy grid.
 * @details Designed for a 3D-capable vehicle while outputting a 2D/2.5D grid.
 */
class OccupancyMapper {
public:
  /**
   * @brief Construct an occupancy mapper.
   * @param grid_res Grid resolution in meters.
   * @param width Grid width in meters.
   * @param height Grid height in meters.
   * @param origin_x World x coordinate of the grid origin.
   * @param origin_y World y coordinate of the grid origin.
   */
  OccupancyMapper(float grid_res, float width, float height,
                  float origin_x, float origin_y);

  /**
   * @brief Destroy the mapper and release GPU resources.
   */
  ~OccupancyMapper();

  /**
   * @brief Set the accepted world-frame height band.
   * @param min_z Minimum z in meters.
   * @param max_z Maximum z in meters.
   */
  void set_height_band(float min_z, float max_z);

  /**
   * @brief Set the uncertainty threshold for depth points.
   * @param max_uncertainty Maximum acceptable uncertainty value.
   */
  void set_uncertainty_threshold(float max_uncertainty);

  /**
   * @brief Enable or disable GPU-side mapping.
   * @param enable True to use CUDA kernels when available.
   */
  void set_use_gpu(bool enable);

  /**
   * @brief Update the occupancy grid from dense depth.
   * @param depth Dense depth result from the network.
   * @param T_wb Transform from world to body.
   * @param T_bc Transform from body to camera.
   * @param K Camera intrinsics.
   * @details Reprojects each pixel into world coordinates and updates occupancy.
   */
  void update_from_depth(const depth::DepthResult &depth,
                         const Eigen::Matrix4f &T_wb,
                         const Eigen::Matrix4f &T_bc,
                         const depth::CameraIntrinsics &K);

  /**
   * @brief Convert the current occupancy grid into a message.
   * @param row_major True to output row-major data ordering.
   * @return Occupancy grid message with metadata populated.
   */
  nature::msg::OccupancyGrid to_msg(bool row_major) const;

private:
  void clear_grid_locked();
  void update_from_depth_cpu(const depth::DepthResult &depth,
                             const Eigen::Matrix4f &T_wb,
                             const Eigen::Matrix4f &T_bc,
                             const depth::CameraIntrinsics &K);
#ifdef NATURE_HAS_CUDA
  void update_from_depth_gpu(const depth::DepthResult &depth,
                             const Eigen::Matrix4f &T_wb,
                             const Eigen::Matrix4f &T_bc,
                             const depth::CameraIntrinsics &K);
  void allocate_gpu_buffers();
  void download_grid_locked() const;
#endif
  int index_from_xy(int x, int y) const;

  float resolution_;
  float width_;
  float height_;
  float origin_x_;
  float origin_y_;
  int cells_x_;
  int cells_y_;
  float min_z_ = -1.0f;
  float max_z_ = 2.0f;
  float max_uncertainty_ = 0.5f;
  int8_t unknown_value_ = -1;
  mutable std::vector<int8_t> data_;
  mutable std::mutex mutex_;
#ifdef NATURE_HAS_CUDA
  mutable int *grid_gpu_ = nullptr;
  cudaStream_t stream_ = nullptr;
  bool gpu_enabled_ = true;
  bool gpu_ready_ = false;
#endif
};

} // namespace mapping
} // namespace perception
} // namespace nature

#endif // NATURE_OCCUPANCY_MAPPER_H
