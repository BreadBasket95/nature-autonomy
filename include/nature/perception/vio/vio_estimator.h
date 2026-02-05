//
// Library-level wrapper around OpenVINS VIO for the nature autonomy stack.
//
#ifndef NATURE_VIO_ESTIMATOR_H
#define NATURE_VIO_ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace nature {
namespace perception {
namespace vio {

/**
 * @brief VIO output snapshot containing pose and sparse landmarks.
 * @details T_wb is the transform from world to body. Landmarks are expressed
 *          in the world frame to simplify downstream projection.
 */
struct VIOUpdate {
  double timestamp = 0.0;
  Eigen::Matrix4f T_wb = Eigen::Matrix4f::Identity();
  std::vector<Eigen::Vector3f> landmarks_w;
};

/**
 * @brief Library-level wrapper for OpenVINS VIO processing.
 * @details This class ingests IMU + monocular images and produces pose and
 *          sparse landmarks without middleware dependencies.
 */
class VIOEstimator {
public:
  /**
   * @brief Construct the estimator and load OpenVINS configuration.
   * @param config_path Path to an OpenVINS YAML config file.
   * @details Starts an internal worker thread to handle measurement ingestion.
   */
  explicit VIOEstimator(const std::string &config_path);

  /**
   * @brief Destroy the estimator and shut down worker resources.
   */
  ~VIOEstimator();

  /**
   * @brief Feed an IMU sample and optional image into the estimator.
   * @param timestamp Measurement time in seconds.
   * @param image Monocular image at the provided timestamp (may be empty).
   * @param accel Linear acceleration in the body frame (m/s^2).
   * @param gyro Angular velocity in the body frame (rad/s).
   * @details Thread-safe; enqueues data for the internal processing thread.
   */
  void feed_measurement(double timestamp,
                        const cv::Mat &image,
                        const Eigen::Vector3f &accel,
                        const Eigen::Vector3f &gyro);

  /**
   * @brief Feed a single IMU measurement into the estimator.
   * @param timestamp Measurement time in seconds.
   * @param accel Linear acceleration in the body frame (m/s^2).
   * @param gyro Angular velocity in the body frame (rad/s).
   * @details Use this for high-rate IMU ingestion with external sync buffers.
   */
  void feed_imu(double timestamp,
                const Eigen::Vector3f &accel,
                const Eigen::Vector3f &gyro);

  /**
   * @brief Feed a monocular image into the estimator.
   * @param timestamp Image capture time in seconds.
   * @param image Monocular image.
   * @details Use this for time-aligned image ingestion after IMU buffering.
   */
  void feed_image(double timestamp,
                  const cv::Mat &image);

  /**
   * @brief Retrieve the most recent VIO update.
   * @param out Output structure populated on success.
   * @return True if an update was available.
   * @details Returns the latest completed update without blocking.
   */
  bool get_latest_update(VIOUpdate &out);

private:
  void set_latest_update(const VIOUpdate &update);

  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::mutex mutex_;
  VIOUpdate latest_update_;
  bool has_update_ = false;
};

} // namespace vio
} // namespace perception
} // namespace nature

#endif // NATURE_VIO_ESTIMATOR_H
