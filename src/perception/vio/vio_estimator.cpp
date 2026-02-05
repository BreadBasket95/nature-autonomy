//
// OpenVINS wrapper implementation for VIO processing.
//
#include "nature/perception/vio/vio_estimator.h"

#include <condition_variable>
#include <deque>
#include <thread>

#ifdef NATURE_HAS_OPENVINS
#include <ov_msckf/VioManager.h>
#include <ov_msckf/State.h>
#endif

namespace nature {
namespace perception {
namespace vio {

namespace {

/**
 * @brief Bundle a single IMU + image measurement for asynchronous processing.
 */
struct MeasurementPacket {
  double timestamp = 0.0;
  cv::Mat image;
  Eigen::Vector3f accel = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
  bool has_image = false;
};

} // namespace

struct VIOEstimator::Impl {
  explicit Impl(VIOEstimator *owner, const std::string &config_path)
      : owner_(owner), config_path_(config_path) {
#ifdef NATURE_HAS_OPENVINS
    // NOTE: OpenVINS provides library-level constructors; adapt this to your
    // installed API surface. This placeholder avoids ROS wrappers entirely.
    vio_manager_ = std::make_shared<ov_msckf::VioManager>(config_path_);
#endif
    worker_ = std::thread(&Impl::process_loop, this);
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      running_ = false;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void enqueue(const MeasurementPacket &packet) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_.push_back(packet);
    }
    queue_cv_.notify_one();
  }

  void process_loop() {
    while (true) {
      MeasurementPacket packet;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [&]() { return !queue_.empty() || !running_; });
        if (!running_ && queue_.empty()) {
          break;
        }
        packet = queue_.front();
        queue_.pop_front();
      }

#ifdef NATURE_HAS_OPENVINS
      if (vio_manager_) {
        // Feed IMU first, then optionally the camera frame.
        vio_manager_->feed_measurement_imu(packet.timestamp,
                                           packet.gyro.cast<double>(),
                                           packet.accel.cast<double>());
        if (packet.has_image && !packet.image.empty()) {
          vio_manager_->feed_measurement_camera(packet.timestamp, packet.image);
        }

        VIOUpdate update;
        update.timestamp = packet.timestamp;
        update.T_wb = Eigen::Matrix4f::Identity();

        // TODO: Replace with OpenVINS state extraction matching your version.
        // The placeholder keeps interfaces wired for downstream consumers.
        if (owner_) {
          owner_->set_latest_update(update);
        }
        continue;
      }
#endif

      // Placeholder output when OpenVINS is not available.
      VIOUpdate update;
      update.timestamp = packet.timestamp;
      update.T_wb = Eigen::Matrix4f::Identity();
      update.landmarks_w.clear();
      if (owner_) {
        owner_->set_latest_update(update);
      }
    }
  }

  VIOEstimator *owner_ = nullptr;
  std::string config_path_;
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<MeasurementPacket> queue_;
  bool running_ = true;

#ifdef NATURE_HAS_OPENVINS
  std::shared_ptr<ov_msckf::VioManager> vio_manager_;
#endif
};

VIOEstimator::VIOEstimator(const std::string &config_path)
    : impl_(new Impl(this, config_path)) {}

VIOEstimator::~VIOEstimator() = default;

void VIOEstimator::feed_measurement(double timestamp,
                                    const cv::Mat &image,
                                    const Eigen::Vector3f &accel,
                                    const Eigen::Vector3f &gyro) {
  if (!impl_) {
    return;
  }
  MeasurementPacket packet;
  packet.timestamp = timestamp;
  packet.image = image;
  packet.accel = accel;
  packet.gyro = gyro;
  packet.has_image = !image.empty();
  impl_->enqueue(packet);
}

bool VIOEstimator::get_latest_update(VIOUpdate &out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_update_) {
    return false;
  }
  out = latest_update_;
  return true;
}

void VIOEstimator::set_latest_update(const VIOUpdate &update) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_update_ = update;
  has_update_ = true;
}

} // namespace vio
} // namespace perception
} // namespace nature
