//
// Integrated VIO + depth completion node for nature autonomy.
//
#include "nature/messaging/message_types.h"
#include "nature/node/node_proxy.h"
#include "nature/perception/depth/depth_net.h"
#include "nature/perception/depth/feature_projector.h"
#include "nature/perception/mapping/occupancy_mapper.h"
#include "nature/perception/vio/vio_estimator.h"
#ifdef NATURE_HAS_ZMQ
#include "nature/transport/icd_adapter.h"
#include "nature/transport/icd_serialization.h"
#include "nature/transport/zmq_transport.h"
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef NATURE_HAS_CUDA
#include <cuda_runtime.h>
#endif

namespace {

struct ImuSample {
  double timestamp = 0.0;
  Eigen::Vector3f accel = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
};

struct Frame {
  double timestamp = 0.0;
  cv::Mat image;
};

/**
 * @brief Synchronization bundle containing a frame and aligned IMU samples.
 */
struct SyncedPacket {
  Frame frame;
  std::vector<ImuSample> imu_samples;
};

/**
 * @brief Thread-safe buffer that aligns IMU samples with camera frames.
 * @details Drops stale frames and waits for IMU coverage around each image.
 */
class TimeSyncBuffer {
public:
  /**
   * @brief Construct the buffer with timing constraints.
   * @param max_buffer_sec Maximum IMU buffer duration in seconds.
   * @param max_time_slop Maximum allowable timestamp mismatch.
   * @param max_imu_gap Maximum expected IMU gap (for interpolation).
   * @param interpolate True to insert an interpolated IMU sample at frame time.
   */
  TimeSyncBuffer(double max_buffer_sec,
                 double max_time_slop,
                 double max_imu_gap,
                 bool interpolate)
      : max_buffer_sec_(max_buffer_sec),
        max_time_slop_(max_time_slop),
        max_imu_gap_(max_imu_gap),
        interpolate_(interpolate) {}

  /**
   * @brief Add a new IMU sample to the buffer.
   * @param sample IMU sample to store.
   */
  void add_imu(ImuSample sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!imu_samples_.empty() && sample.timestamp < imu_samples_.back().timestamp) {
      // TODO: Handle out-of-order IMU samples (e.g., insert sorted or drop).
    }
    imu_samples_.push_back(std::move(sample));
    prune_imu_locked();
    cv_.notify_all();
  }

  /**
   * @brief Add a new frame to the buffer.
   * @param frame Frame to store.
   */
  void add_frame(Frame frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!frames_.empty() && frame.timestamp < frames_.back().timestamp) {
      // TODO: Handle out-of-order frames (e.g., insert sorted or drop).
    }
    frames_.push_back(std::move(frame));
    while (frames_.size() > 1 &&
           (frames_.back().timestamp - frames_.front().timestamp) > max_buffer_sec_) {
      // TODO: Decide whether to drop oldest frames or pause ingestion.
      frames_.pop_front();
    }
    cv_.notify_all();
  }

  /**
   * @brief Retrieve the next time-aligned packet.
   * @param out Output packet containing IMU samples and an image.
   * @return True if a packet was produced; false on shutdown.
   */
  bool get_synced(SyncedPacket &out) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&]() { return shutdown_ || (!frames_.empty() && !imu_samples_.empty()); });
    if (shutdown_) {
      return false;
    }

    while (!frames_.empty()) {
      const double frame_ts = frames_.front().timestamp;
      prune_imu_locked();

      if (imu_samples_.empty()) {
        return false;
      }

      if (frame_ts + max_time_slop_ < imu_samples_.front().timestamp) {
        // Frame is too old; drop it.
        frames_.pop_front();
        continue;
      }

      if (frame_ts > imu_samples_.back().timestamp + max_time_slop_) {
        // Not enough IMU data yet; wait for more.
        return false;
      }

      out.imu_samples.clear();
      while (!imu_samples_.empty() && imu_samples_.front().timestamp <= frame_ts) {
        out.imu_samples.push_back(imu_samples_.front());
        imu_samples_.pop_front();
      }

      if (interpolate_ && !out.imu_samples.empty() && !imu_samples_.empty()) {
        const ImuSample &before = out.imu_samples.back();
        const ImuSample &after = imu_samples_.front();
        const double dt = after.timestamp - before.timestamp;
        if (dt > 1e-6 && dt < max_imu_gap_) {
          const double alpha = (frame_ts - before.timestamp) / dt;
          if (alpha > 0.0 && alpha < 1.0) {
            ImuSample interp;
            interp.timestamp = frame_ts;
            interp.accel = before.accel + static_cast<float>(alpha) * (after.accel - before.accel);
            interp.gyro = before.gyro + static_cast<float>(alpha) * (after.gyro - before.gyro);
            out.imu_samples.push_back(interp);
          }
        } else if (dt >= max_imu_gap_) {
          // TODO: Handle large IMU gaps with prediction or sensor fault reporting.
        }
      }

      out.frame = frames_.front();
      frames_.pop_front();
      return true;
    }

    return false;
  }

  /**
   * @brief Signal all waiting threads to shut down.
   */
  void shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
    cv_.notify_all();
  }

private:
  void prune_imu_locked() {
    if (imu_samples_.empty()) {
      return;
    }
    const double newest = imu_samples_.back().timestamp;
    while (!imu_samples_.empty() && (newest - imu_samples_.front().timestamp) > max_buffer_sec_) {
      imu_samples_.pop_front();
    }
  }

  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Frame> frames_;
  std::deque<ImuSample> imu_samples_;
  bool shutdown_ = false;
  double max_buffer_sec_ = 2.0;
  double max_time_slop_ = 0.02;
  double max_imu_gap_ = 0.05;
  bool interpolate_ = true;
};



/**
 * @brief Trim whitespace from both ends of a string.
 * @param input String to trim.
 * @return Trimmed string.
 */
std::string trim(const std::string &input) {
  const auto start = input.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(start, end - start + 1);
}

/**
 * @brief Parse a YAML line into key/value tokens.
 * @param line Input line.
 * @param key Output key.
 * @param value Output value.
 * @return True if a key/value pair was extracted.
 */
bool parse_yaml_line(const std::string &line, std::string &key, std::string &value) {
  const std::string cleaned = trim(line);
  if (cleaned.empty() || cleaned[0] == '#') {
    return false;
  }
  const auto pos = cleaned.find(':');
  if (pos == std::string::npos) {
    return false;
  }
  key = trim(cleaned.substr(0, pos));
  value = trim(cleaned.substr(pos + 1));
  return !key.empty();
}

/**
 * @brief Apply a YAML value to the node parameter store.
 * @param node NodeProxy to mutate.
 * @param key Parameter key.
 * @param value Parameter value string.
 */
void apply_yaml_parameter(nature::node::NodeProxy &node,
                          const std::string &key,
                          const std::string &value) {
  if (value.empty()) {
    return;
  }
  if (value.front() == '"' && value.back() == '"') {
    node.set_parameter(key, value.substr(1, value.size() - 2));
    return;
  }
  if (value == "true" || value == "false") {
    node.set_parameter(key, value == "true");
    return;
  }
  if (value.front() == '[' && value.back() == ']') {
    std::vector<double> list;
    std::string inner = value.substr(1, value.size() - 2);
    size_t start = 0;
    while (start < inner.size()) {
      const size_t comma = inner.find(',', start);
      const std::string token = trim(inner.substr(start, comma - start));
      if (!token.empty()) {
        list.push_back(std::stod(token));
      }
      if (comma == std::string::npos) {
        break;
      }
      start = comma + 1;
    }
    node.set_parameter(key, list);
    return;
  }

  char *endptr = nullptr;
  const double numeric = std::strtod(value.c_str(), &endptr);
  if (endptr && *endptr == '\0') {
    const bool has_decimal = value.find('.') != std::string::npos ||
                             value.find('e') != std::string::npos ||
                             value.find('E') != std::string::npos;
    if (has_decimal) {
      node.set_parameter(key, numeric);
    } else {
      node.set_parameter(key, static_cast<int>(numeric));
    }
    return;
  }

  node.set_parameter(key, value);
}

/**
 * @brief Load a flat YAML file into the node parameter map.
 * @param path YAML file path.
 * @param node NodeProxy to populate.
 * @return True if the file was read successfully.
 */
bool load_yaml_config(const std::string &path, nature::node::NodeProxy &node) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    std::string key;
    std::string value;
    if (parse_yaml_line(line, key, value)) {
      apply_yaml_parameter(node, key, value);
    }
  }
  return true;
}

/**
 * @brief Convert a placeholder image message into an OpenCV Mat.
 * @param msg Input image message.
 * @param out Output image in BGR or grayscale.
 * @return True if the conversion succeeded.
 */
bool image_msg_to_cv(const nature::msg::Image &msg, cv::Mat &out) {
  if (msg.height == 0 || msg.width == 0 || msg.data.empty()) {
    return false;
  }

  const std::string encoding = trim(msg.encoding);
  if (encoding == "mono8") {
    if (msg.data.size() < msg.height * msg.width) {
      return false;
    }
    out = cv::Mat(static_cast<int>(msg.height),
                  static_cast<int>(msg.width),
                  CV_8UC1,
                  const_cast<uint8_t *>(msg.data.data()),
                  msg.step > 0 ? static_cast<size_t>(msg.step) : msg.width).clone();
    return true;
  }

  if (encoding == "rgb8" || encoding == "bgr8") {
    const size_t expected = static_cast<size_t>(msg.height * msg.width * 3);
    if (msg.data.size() < expected) {
      return false;
    }
    cv::Mat tmp(static_cast<int>(msg.height),
                static_cast<int>(msg.width),
                CV_8UC3,
                const_cast<uint8_t *>(msg.data.data()),
                msg.step > 0 ? static_cast<size_t>(msg.step) : msg.width * 3);
    if (encoding == "rgb8") {
      cv::cvtColor(tmp, out, cv::COLOR_RGB2BGR);
    } else {
      out = tmp.clone();
    }
    return true;
  }

  // TODO: Add support for additional encodings (e.g., mono16, rgba8, compressed).
  return false;
}

/**
 * @brief Fill a CPU RGB buffer in NCHW float layout.
 * @param image Input image (BGR or grayscale).
 * @param width Output width.
 * @param height Output height.
 * @param out Output buffer (size width*height*3).
 */
void fill_rgb_buffer(const cv::Mat &image,
                     int width,
                     int height,
                     const std::array<float, 3> &mean,
                     const std::array<float, 3> &stddev,
                     float scale,
                     std::vector<float> &out) {
  out.assign(static_cast<size_t>(width * height * 3), 0.0f);
  if (image.empty()) {
    return;
  }

  cv::Mat resized;
  if (image.cols != width || image.rows != height) {
    cv::resize(image, resized, cv::Size(width, height));
  } else {
    resized = image;
  }

  const int channels = resized.channels();
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float r = 0.0f;
      float g = 0.0f;
      float b = 0.0f;
      if (channels == 1) {
        const uint8_t value = resized.at<uint8_t>(v, u);
        const float scaled = static_cast<float>(value) * scale;
        r = g = b = scaled;
      } else {
        const cv::Vec3b pixel = resized.at<cv::Vec3b>(v, u);
        b = static_cast<float>(pixel[0]) * scale;
        g = static_cast<float>(pixel[1]) * scale;
        r = static_cast<float>(pixel[2]) * scale;
      }
      const int idx = v * width + u;
      out[idx] = (r - mean[0]) / stddev[0];
      out[idx + width * height] = (g - mean[1]) / stddev[1];
      out[idx + 2 * width * height] = (b - mean[2]) / stddev[2];
    }
  }
}

/**
 * @brief Reorder an RGB buffer from NCHW to NHWC layout.
 * @param nchw Input buffer in NCHW layout.
 * @param width Image width.
 * @param height Image height.
 * @param nhwc Output buffer in NHWC layout.
 */
void reorder_nchw_to_nhwc(const std::vector<float> &nchw,
                          int width,
                          int height,
                          std::vector<float> &nhwc) {
  const size_t plane = static_cast<size_t>(width * height);
  nhwc.assign(plane * 3, 0.0f);
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const size_t idx = static_cast<size_t>(v * width + u);
      nhwc[idx * 3 + 0] = nchw[idx];
      nhwc[idx * 3 + 1] = nchw[idx + plane];
      nhwc[idx * 3 + 2] = nchw[idx + 2 * plane];
    }
  }
}

/**
 * @brief Convert a VIO update into an odometry message.
 * @param vio VIO update containing world->body transform.
 * @param stamp Timestamp for the message.
 * @return Odometry message in the world frame.
 */
nature::msg::Odometry vio_to_odometry(const nature::perception::vio::VIOUpdate &vio,
                                      double stamp) {
  nature::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "map";

  const Eigen::Matrix4f T_bw = vio.T_wb.inverse();
  const Eigen::Matrix3f R_bw = T_bw.block<3, 3>(0, 0);
  const Eigen::Quaternionf q(R_bw);

  odom.pose.pose.position.x = T_bw(0, 3);
  odom.pose.pose.position.y = T_bw(1, 3);
  odom.pose.pose.position.z = T_bw(2, 3);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  return odom;
}

} // namespace

int main(int argc, char *argv[]) {
  auto node = nature::node::init_node(argc, argv, "nature_vio_depth_node");

  std::string config_path = "config/vio_depth.yaml";
  node->get_parameter("~config_path", config_path, config_path);
  load_yaml_config(config_path, *node);

  // Camera parameters
  double fx = 320.0;
  double fy = 320.0;
  double cx = 320.0;
  double cy = 240.0;
  int camera_width = 640;
  int camera_height = 480;
  node->get_parameter("camera.fx", fx, fx);
  node->get_parameter("camera.fy", fy, fy);
  node->get_parameter("camera.cx", cx, cx);
  node->get_parameter("camera.cy", cy, cy);
  node->get_parameter("camera.width", camera_width, camera_width);
  node->get_parameter("camera.height", camera_height, camera_height);

  bool synthetic_camera = true;
  bool synthetic_imu = true;
  double camera_rate_hz = 30.0;
  double imu_rate_hz = 200.0;
  node->get_parameter("synthetic_camera", synthetic_camera, synthetic_camera);
  node->get_parameter("synthetic_imu", synthetic_imu, synthetic_imu);
  node->get_parameter("camera.rate_hz", camera_rate_hz, camera_rate_hz);
  node->get_parameter("imu.rate_hz", imu_rate_hz, imu_rate_hz);

  std::string openvins_config = "config/openvins.yaml";
  std::string onnx_path = "models/iudc.onnx";
  std::string engine_path;
  bool tensorrt_fp16 = true;
  node->get_parameter("openvins.config", openvins_config, openvins_config);
  node->get_parameter("tensorrt.onnx_path", onnx_path, onnx_path);
  node->get_parameter("tensorrt.engine_path", engine_path, engine_path);
  node->get_parameter("tensorrt.fp16", tensorrt_fp16, tensorrt_fp16);
  if (engine_path.empty()) {
    engine_path = onnx_path + ".engine";
  }

  double rgb_scale = 1.0 / 255.0;
  std::vector<double> rgb_mean_vec{0.0, 0.0, 0.0};
  std::vector<double> rgb_std_vec{1.0, 1.0, 1.0};
  node->get_parameter("rgb.scale", rgb_scale, rgb_scale);
  node->get_parameter("rgb.mean", rgb_mean_vec, rgb_mean_vec);
  node->get_parameter("rgb.std", rgb_std_vec, rgb_std_vec);
  std::array<float, 3> rgb_mean = {static_cast<float>(rgb_mean_vec.size() > 0 ? rgb_mean_vec[0] : 0.0),
                                   static_cast<float>(rgb_mean_vec.size() > 1 ? rgb_mean_vec[1] : 0.0),
                                   static_cast<float>(rgb_mean_vec.size() > 2 ? rgb_mean_vec[2] : 0.0)};
  std::array<float, 3> rgb_std = {static_cast<float>(rgb_std_vec.size() > 0 ? rgb_std_vec[0] : 1.0),
                                  static_cast<float>(rgb_std_vec.size() > 1 ? rgb_std_vec[1] : 1.0),
                                  static_cast<float>(rgb_std_vec.size() > 2 ? rgb_std_vec[2] : 1.0)};

  double sync_buffer_sec = 2.0;
  double sync_time_slop = 0.02;
  double sync_max_imu_gap = 0.05;
  bool sync_interpolate = true;
  node->get_parameter("sync.buffer_sec", sync_buffer_sec, sync_buffer_sec);
  node->get_parameter("sync.time_slop", sync_time_slop, sync_time_slop);
  node->get_parameter("sync.max_imu_gap", sync_max_imu_gap, sync_max_imu_gap);
  node->get_parameter("sync.interpolate", sync_interpolate, sync_interpolate);

  bool zmq_enable = false;
  std::string zmq_pub_endpoint = "tcp://*:5556";
  std::string zmq_sub_endpoint = "tcp://localhost:5557";
  node->get_parameter("transport.zmq_enable", zmq_enable, zmq_enable);
  node->get_parameter("transport.zmq_pub_endpoint", zmq_pub_endpoint, zmq_pub_endpoint);
  node->get_parameter("transport.zmq_sub_endpoint", zmq_sub_endpoint, zmq_sub_endpoint);

  // Grid parameters
  double grid_res = 0.5;
  double grid_width = 200.0;
  double grid_height = 200.0;
  double grid_origin_x = -100.0;
  double grid_origin_y = -100.0;
  node->get_parameter("grid.resolution", grid_res, grid_res);
  node->get_parameter("grid.width", grid_width, grid_width);
  node->get_parameter("grid.height", grid_height, grid_height);
  node->get_parameter("grid.origin_x", grid_origin_x, grid_origin_x);
  node->get_parameter("grid.origin_y", grid_origin_y, grid_origin_y);

  double min_z = -1.0;
  double max_z = 3.0;
  double max_uncertainty = 0.5;
  node->get_parameter("height_band.min_z", min_z, min_z);
  node->get_parameter("height_band.max_z", max_z, max_z);
  node->get_parameter("uncertainty.max", max_uncertainty, max_uncertainty);

  // Parse extrinsics
  Eigen::Matrix4f T_bc = Eigen::Matrix4f::Identity();
  std::vector<double> t_bc_values;
  if (node->get_parameter("t_bc", t_bc_values, std::vector<double>()) && t_bc_values.size() >= 16) {
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        T_bc(r, c) = static_cast<float>(t_bc_values[static_cast<size_t>(r * 4 + c)]);
      }
    }
  }

  nature::perception::depth::CameraIntrinsics K;
  K.fx = static_cast<float>(fx);
  K.fy = static_cast<float>(fy);
  K.cx = static_cast<float>(cx);
  K.cy = static_cast<float>(cy);
  K.width = camera_width;
  K.height = camera_height;

  nature::perception::vio::VIOEstimator vio(openvins_config);
  nature::perception::depth::DepthNet depth_net;
  nature::perception::depth::FeatureProjector feature_projector;
  const bool depth_ready = depth_net.load_engine(onnx_path, engine_path, tensorrt_fp16);
  if (!depth_ready) {
    std::cerr << "DepthNet failed to load; running in placeholder mode." << std::endl;
  }
  const bool rgb_expect_nhwc = depth_net.expects_rgb_nhwc();
  const bool sparse_expect_nhwc = depth_net.expects_sparse_nhwc();
  if (sparse_expect_nhwc) {
    // TODO: Handle sparse depth NHWC reordering for multi-channel inputs.
  }

  nature::perception::mapping::OccupancyMapper mapper(
      static_cast<float>(grid_res),
      static_cast<float>(grid_width),
      static_cast<float>(grid_height),
      static_cast<float>(grid_origin_x),
      static_cast<float>(grid_origin_y));
  mapper.set_height_band(static_cast<float>(min_z), static_cast<float>(max_z));
  mapper.set_uncertainty_threshold(static_cast<float>(max_uncertainty));

  auto grid_pub = node->create_publisher<nature::msg::OccupancyGrid>("nature/occupancy_grid", 1);
  auto odom_pub = node->create_publisher<nature::msg::Odometry>("nature/odometry", 1);

  TimeSyncBuffer sync_buffer(sync_buffer_sec, sync_time_slop, sync_max_imu_gap, sync_interpolate);

#ifdef NATURE_HAS_ZMQ
  std::shared_ptr<nature::transport::ZmqTransport> transport;
  if (zmq_enable) {
    transport = std::make_shared<nature::transport::ZmqTransport>(zmq_pub_endpoint, zmq_sub_endpoint);
    transport->subscribe("icd/imu", [&](const std::vector<uint8_t> &payload) {
      nature::icd::Imu icd_imu;
      if (!nature::transport::deserialize(payload, icd_imu)) {
        return;
      }
      const auto msg = nature::transport::to_msg(icd_imu);
      ImuSample sample;
      sample.timestamp = msg.header.stamp;
      sample.accel = Eigen::Vector3f(static_cast<float>(msg.linear_acceleration.x),
                                     static_cast<float>(msg.linear_acceleration.y),
                                     static_cast<float>(msg.linear_acceleration.z));
      sample.gyro = Eigen::Vector3f(static_cast<float>(msg.angular_velocity.x),
                                    static_cast<float>(msg.angular_velocity.y),
                                    static_cast<float>(msg.angular_velocity.z));
      sync_buffer.add_imu(sample);
    });
    transport->subscribe("icd/image", [&](const std::vector<uint8_t> &payload) {
      nature::icd::Image icd_image;
      if (!nature::transport::deserialize(payload, icd_image)) {
        return;
      }
      const auto msg = nature::transport::to_msg(icd_image);
      Frame frame;
      frame.timestamp = msg.header.stamp;
      if (image_msg_to_cv(msg, frame.image)) {
        sync_buffer.add_frame(std::move(frame));
      }
    });
    // TODO: Add additional ICD subscriptions for other sensors (GPS, lidar, etc.).
  }
#endif

  auto imu_sub = node->create_subscription<nature::msg::Imu>(
      "nature/imu", 1,
      [&](nature::msg::ImuPtr msg) {
        ImuSample sample;
        sample.timestamp = msg->header.stamp;
        sample.accel = Eigen::Vector3f(static_cast<float>(msg->linear_acceleration.x),
                                       static_cast<float>(msg->linear_acceleration.y),
                                       static_cast<float>(msg->linear_acceleration.z));
        sample.gyro = Eigen::Vector3f(static_cast<float>(msg->angular_velocity.x),
                                      static_cast<float>(msg->angular_velocity.y),
                                      static_cast<float>(msg->angular_velocity.z));
        sync_buffer.add_imu(sample);
      });
  (void)imu_sub;

  auto image_sub = node->create_subscription<nature::msg::Image>(
      "nature/camera/image", 1,
      [&](nature::msg::ImagePtr msg) {
        Frame frame;
        frame.timestamp = msg->header.stamp;
        if (image_msg_to_cv(*msg, frame.image)) {
          sync_buffer.add_frame(std::move(frame));
        }
      });
  (void)image_sub;

  std::atomic<bool> running(true);

  std::thread zmq_thread;
#ifdef NATURE_HAS_ZMQ
  if (transport) {
    zmq_thread = std::thread([&]() {
      while (running.load()) {
        transport->poll_once(10);
      }
    });
  }
#endif

  std::thread imu_thread;
  if (synthetic_imu) {
    imu_thread = std::thread([&]() {
      nature::node::Rate rate(imu_rate_hz);
      while (running.load()) {
        ImuSample sample;
        sample.timestamp = node->get_now_seconds();
        sample.accel = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        sample.gyro = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        sync_buffer.add_imu(sample);
        rate.sleep();
      }
    });
  }

  std::thread camera_thread;
  if (synthetic_camera) {
    camera_thread = std::thread([&]() {
      nature::node::Rate rate(camera_rate_hz);
      while (running.load()) {
        Frame frame;
        frame.timestamp = node->get_now_seconds();
        frame.image = cv::Mat(camera_height, camera_width, CV_8UC3, cv::Scalar(0, 0, 0));
        sync_buffer.add_frame(std::move(frame));
        rate.sleep();
      }
    });
  }

#ifdef NATURE_HAS_CUDA
  cudaStream_t stream = nullptr;
  cudaStreamCreate(&stream);
#else
  cudaStream_t stream = nullptr;
#endif

  std::vector<float> rgb_cpu;
  std::vector<float> rgb_cpu_nhwc;
  std::vector<float> sparse_cpu;
  float *rgb_gpu = nullptr;
  float *sparse_gpu = nullptr;
  bool use_gpu = false;

#ifdef NATURE_HAS_CUDA
  use_gpu = depth_ready;
  if (use_gpu) {
    cudaMalloc(&rgb_gpu, static_cast<size_t>(camera_width * camera_height * 3) * sizeof(float));
    cudaMalloc(&sparse_gpu, static_cast<size_t>(camera_width * camera_height) * sizeof(float));
  }
#endif

  if (!use_gpu) {
    rgb_cpu.resize(static_cast<size_t>(camera_width * camera_height * 3), 0.0f);
    sparse_cpu.resize(static_cast<size_t>(camera_width * camera_height), 0.0f);
    if (rgb_expect_nhwc) {
      rgb_cpu_nhwc.resize(static_cast<size_t>(camera_width * camera_height * 3), 0.0f);
      rgb_gpu = rgb_cpu_nhwc.data();
    } else {
      rgb_gpu = rgb_cpu.data();
    }
    sparse_gpu = sparse_cpu.data();
  }

  mapper.set_use_gpu(use_gpu);

  std::thread depth_thread([&]() {
    while (running.load()) {
      SyncedPacket packet;
      if (!sync_buffer.get_synced(packet)) {
        if (!running.load()) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      if (packet.imu_samples.empty()) {
        // TODO: Decide whether to drop frames without IMU data or extrapolate.
        continue;
      }

      for (const auto &imu_sample : packet.imu_samples) {
        vio.feed_imu(imu_sample.timestamp, imu_sample.accel, imu_sample.gyro);
      }
      vio.feed_image(packet.frame.timestamp, packet.frame.image);

      nature::perception::vio::VIOUpdate vio_update;
      if (!vio.get_latest_update(vio_update)) {
        continue;
      }

      fill_rgb_buffer(packet.frame.image, camera_width, camera_height, rgb_mean, rgb_std,
                      static_cast<float>(rgb_scale), rgb_cpu);
      if (rgb_expect_nhwc) {
        reorder_nchw_to_nhwc(rgb_cpu, camera_width, camera_height, rgb_cpu_nhwc);
      }
      const float *rgb_host_ptr = rgb_expect_nhwc ? rgb_cpu_nhwc.data() : rgb_cpu.data();
#ifdef NATURE_HAS_CUDA
      if (use_gpu) {
        cudaMemcpyAsync(rgb_gpu, rgb_host_ptr,
                        static_cast<size_t>(camera_width * camera_height * 3) * sizeof(float),
                        cudaMemcpyHostToDevice, stream);
      }
#endif

      feature_projector.generate_sparse_depth_map(
          vio_update, T_bc, K, sparse_gpu, stream);

      nature::perception::depth::DepthResult depth_result;
      bool have_depth = false;
      if (depth_ready && depth_net.infer(rgb_gpu, sparse_gpu, camera_width, camera_height, depth_result, stream)) {
        have_depth = true;
#ifdef NATURE_HAS_CUDA
        if (use_gpu) {
          cudaStreamSynchronize(stream);
        }
#endif
      } else {
        // Placeholder: treat sparse depth as dense when the network is unavailable.
        depth_result.dense_depth_gpu = sparse_gpu;
        depth_result.uncertainty_gpu = nullptr;
        depth_result.width = K.width;
        depth_result.height = K.height;
        have_depth = true;
      }

      if (have_depth) {
        mapper.update_from_depth(depth_result, vio_update.T_wb, T_bc, K);
        auto grid_msg = mapper.to_msg(true);
        grid_msg.header.stamp = node->get_now_seconds();
        grid_pub->publish(grid_msg);
#ifdef NATURE_HAS_ZMQ
        if (transport) {
          const auto icd_grid = nature::transport::to_icd(grid_msg);
          transport->publish("icd/occupancy_grid", nature::transport::serialize(icd_grid));
        }
#endif
      }

      auto odom_msg = vio_to_odometry(vio_update, node->get_now_seconds());
      odom_pub->publish(odom_msg);
#ifdef NATURE_HAS_ZMQ
      if (transport) {
        const auto icd_odom = nature::transport::to_icd(odom_msg);
        transport->publish("icd/odometry", nature::transport::serialize(icd_odom));
      }
#endif
    }
  });

  while (nature::node::ok()) {
    node->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  running.store(false);
  sync_buffer.shutdown();
  if (imu_thread.joinable()) {
    imu_thread.join();
  }
  if (camera_thread.joinable()) {
    camera_thread.join();
  }
  if (depth_thread.joinable()) {
    depth_thread.join();
  }
  if (zmq_thread.joinable()) {
    zmq_thread.join();
  }

#ifdef NATURE_HAS_CUDA
  if (use_gpu) {
    cudaFree(rgb_gpu);
    cudaFree(sparse_gpu);
  }
  if (stream) {
    cudaStreamDestroy(stream);
  }
#endif

  return 0;
}
