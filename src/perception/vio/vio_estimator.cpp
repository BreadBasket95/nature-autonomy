//
// OpenVINS wrapper implementation for VIO processing.
//
#include "nature/perception/vio/vio_estimator.h"

#include <condition_variable>
#include <deque>
#include <thread>
#include <unordered_map>

#ifdef NATURE_HAS_OPENVINS
#include <ov_core/CameraData.h>
#include <ov_core/ImuData.h>
#include <ov_core/YamlParser.h>
#include <ov_msckf/VioManagerOptions.h>
#include <ov_msckf/VioManager.h>
#include <ov_msckf/State.h>
#include <ov_type/IMU.h>
#include <opencv2/imgproc.hpp>
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
  bool has_imu = false;
  bool has_image = false;
};

#ifdef NATURE_HAS_OPENVINS
template <typename ManagerT>
auto feed_imu(ManagerT *manager, const ov_core::ImuData &imu, int)
    -> decltype(manager->feed_measurement_imu(imu), void()) {
  manager->feed_measurement_imu(imu);
}

template <typename ManagerT>
auto feed_imu(ManagerT *manager, const ov_core::ImuData &imu, long)
    -> decltype(manager->feed_measurement_imu(imu.timestamp, imu.wm, imu.am), void()) {
  manager->feed_measurement_imu(imu.timestamp, imu.wm, imu.am);
}

template <typename ManagerT>
auto feed_camera(ManagerT *manager, const ov_core::CameraData &cam, int)
    -> decltype(manager->feed_measurement_camera(cam), void()) {
  manager->feed_measurement_camera(cam);
}

template <typename ManagerT>
auto feed_camera(ManagerT *manager, const ov_core::CameraData &cam, long)
    -> decltype(manager->feed_measurement_monocular(cam.timestamp, cam.images.at(0), static_cast<size_t>(0)), void()) {
  if (!cam.images.empty()) {
    manager->feed_measurement_monocular(cam.timestamp, cam.images.at(0), static_cast<size_t>(0));
  }
}

template <typename ManagerT>
auto get_msckf_features(ManagerT *manager, int) -> decltype(manager->get_good_features_MSCKF()) {
  return manager->get_good_features_MSCKF();
}

template <typename ManagerT>
void get_active_tracks(ManagerT *manager,
                       double &timestamp,
                       std::unordered_map<size_t, Eigen::Vector3d> &feat_pos,
                       std::unordered_map<size_t, Eigen::Vector3d> &feat_tracks,
                       int) {
  manager->get_active_tracks(timestamp, feat_pos, feat_tracks);
}

template <typename ManagerT>
void get_active_tracks(ManagerT *,
                       double &,
                       std::unordered_map<size_t, Eigen::Vector3d> &,
                       std::unordered_map<size_t, Eigen::Vector3d> &,
                       long) {}

template <typename ManagerT>
auto get_msckf_features(ManagerT *manager, long) -> std::vector<Eigen::Vector3d> {
  std::vector<Eigen::Vector3d> out;
  std::unordered_map<size_t, Eigen::Vector3d> feat_pos;
  std::unordered_map<size_t, Eigen::Vector3d> feat_tracks;
  double timestamp = 0.0;
  get_active_tracks(manager, timestamp, feat_pos, feat_tracks, 0);
  out.reserve(feat_pos.size());
  for (const auto &kv : feat_pos) {
    out.push_back(kv.second);
  }
  return out;
}

template <typename ValueT>
auto unwrap_feature(const ValueT &value, int) -> decltype(value.second) {
  return value.second;
}

template <typename ValueT>
auto unwrap_feature(const ValueT &value, long) -> decltype(value) {
  return value;
}

template <typename FeatureT>
auto try_feature_pos(const FeatureT &feature, Eigen::Vector3d &out, int)
    -> decltype((void)feature->p_FinG, bool()) {
  out = feature->p_FinG;
  return true;
}

template <typename FeatureT>
auto try_feature_pos(const FeatureT &feature, Eigen::Vector3d &out, long)
    -> decltype((void)feature->get_xyz(), bool()) {
  out = feature->get_xyz();
  return true;
}

template <typename FeatureT>
bool try_feature_pos(const FeatureT &, Eigen::Vector3d &, ...) {
  return false;
}

template <typename StateT>
auto extract_features_from_state(StateT *state, int) -> decltype(state->features_in_view(), std::vector<Eigen::Vector3d>()) {
  std::vector<Eigen::Vector3d> out;
  const auto features = state->features_in_view();
  for (const auto &item : features) {
    const auto feature = unwrap_feature(item, 0);
    Eigen::Vector3d pos;
    if (try_feature_pos(feature, pos, 0)) {
      out.push_back(pos);
    }
  }
  return out;
}

template <typename StateT>
auto extract_features_from_state(StateT *state, long) -> decltype(state->get_features_in_view(), std::vector<Eigen::Vector3d>()) {
  std::vector<Eigen::Vector3d> out;
  const auto features = state->get_features_in_view();
  for (const auto &item : features) {
    const auto feature = unwrap_feature(item, 0);
    Eigen::Vector3d pos;
    if (try_feature_pos(feature, pos, 0)) {
      out.push_back(pos);
    }
  }
  return out;
}

template <typename StateT>
std::vector<Eigen::Vector3d> extract_features_from_state(StateT *, ...) {
  return {};
}
#endif

} // namespace

struct VIOEstimator::Impl {
  explicit Impl(VIOEstimator *owner, const std::string &config_path)
      : owner_(owner), config_path_(config_path) {
#ifdef NATURE_HAS_OPENVINS
    auto parser = std::make_shared<ov_core::YamlParser>(config_path_, false);
    ov_msckf::VioManagerOptions params;
    params.print_and_load(parser);
    vio_manager_ = std::make_shared<ov_msckf::VioManager>(params);
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
        if (packet.has_imu) {
          ov_core::ImuData imu;
          imu.timestamp = packet.timestamp;
          imu.wm = packet.gyro.cast<double>();
          imu.am = packet.accel.cast<double>();
          feed_imu(vio_manager_.get(), imu, 0);
        }

        if (packet.has_image && !packet.image.empty()) {
          ov_core::CameraData cam;
          cam.timestamp = packet.timestamp;
          cam.sensor_ids.push_back(0);
          cv::Mat gray;
          if (packet.image.channels() == 1) {
            gray = packet.image;
          } else {
            cv::cvtColor(packet.image, gray, cv::COLOR_BGR2GRAY);
          }
          cam.images.push_back(gray);
          feed_camera(vio_manager_.get(), cam, 0);
        }

        VIOUpdate update;
        update.timestamp = packet.timestamp;
        update.T_wb = Eigen::Matrix4f::Identity();
        update.landmarks_w.clear();

        if (vio_manager_->initialized()) {
          const auto state = vio_manager_->get_state();
          if (state && state->imu()) {
            const Eigen::Matrix3d R_GtoI = state->imu()->Rot();
            const Eigen::Vector3d p_IinG = state->imu()->pos();
            const Eigen::Matrix3f R = R_GtoI.cast<float>();
            const Eigen::Vector3f p = p_IinG.cast<float>();
            update.T_wb.block<3, 3>(0, 0) = R;
            update.T_wb.block<3, 1>(0, 3) = -R * p;
          }

          std::vector<Eigen::Vector3d> features;
          if (state) {
            features = extract_features_from_state(state.get(), 0);
          }
          if (features.empty()) {
            features = get_msckf_features(vio_manager_.get(), 0);
          }
          if (features.empty()) {
            // TODO: Verify feature extraction against your OpenVINS version and StateServer API.
          }
          update.landmarks_w.reserve(features.size());
          for (const auto &feat : features) {
            update.landmarks_w.emplace_back(static_cast<float>(feat.x()),
                                            static_cast<float>(feat.y()),
                                            static_cast<float>(feat.z()));
          }
          update.timestamp = state ? state->timestamp() : packet.timestamp;
        }

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
  packet.has_imu = true;
  packet.has_image = !image.empty();
  impl_->enqueue(packet);
}

void VIOEstimator::feed_imu(double timestamp,
                            const Eigen::Vector3f &accel,
                            const Eigen::Vector3f &gyro) {
  if (!impl_) {
    return;
  }
  MeasurementPacket packet;
  packet.timestamp = timestamp;
  packet.accel = accel;
  packet.gyro = gyro;
  packet.has_imu = true;
  packet.has_image = false;
  impl_->enqueue(packet);
}

void VIOEstimator::feed_image(double timestamp,
                              const cv::Mat &image) {
  if (!impl_) {
    return;
  }
  MeasurementPacket packet;
  packet.timestamp = timestamp;
  packet.image = image;
  packet.has_imu = false;
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
