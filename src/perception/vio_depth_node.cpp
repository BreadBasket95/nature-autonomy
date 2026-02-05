//
// Integrated VIO + depth completion node for nature autonomy.
//
#include "nature/messaging/message_types.h"
#include "nature/node/node_proxy.h"
#include "nature/perception/depth/depth_net.h"
#include "nature/perception/depth/feature_projector.h"
#include "nature/perception/mapping/occupancy_mapper.h"
#include "nature/perception/vio/vio_estimator.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef NATURE_HAS_CUDA
#include <cuda_runtime.h>
#endif

namespace {

struct Frame {
  double timestamp = 0.0;
  cv::Mat image;
};

/**
 * @brief Thread-safe queue for camera frames.
 */
class FrameQueue {
public:
  /**
   * @brief Push a new frame into the queue.
   * @param frame Frame to store.
   */
  void push(Frame frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.push_back(std::move(frame));
    cv_.notify_one();
  }

  /**
   * @brief Block until a frame is available or shutdown is requested.
   * @param out Output frame.
   * @return True if a frame was returned; false on shutdown.
   */
  bool wait_pop(Frame &out) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&]() { return !frames_.empty() || shutdown_; });
    if (shutdown_) {
      return false;
    }
    out = std::move(frames_.front());
    frames_.pop_front();
    return true;
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
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Frame> frames_;
  bool shutdown_ = false;
};

struct ImuSample {
  double timestamp = 0.0;
  Eigen::Vector3f accel = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
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
 * @brief Fill a CPU RGB buffer in NCHW float layout.
 * @param image Input image (BGR or grayscale).
 * @param width Output width.
 * @param height Output height.
 * @param out Output buffer (size width*height*3).
 */
void fill_rgb_buffer(const cv::Mat &image,
                     int width,
                     int height,
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
        r = g = b = static_cast<float>(value) / 255.0f;
      } else {
        const cv::Vec3b pixel = resized.at<cv::Vec3b>(v, u);
        b = static_cast<float>(pixel[0]) / 255.0f;
        g = static_cast<float>(pixel[1]) / 255.0f;
        r = static_cast<float>(pixel[2]) / 255.0f;
      }
      const int idx = v * width + u;
      out[idx] = r;
      out[idx + width * height] = g;
      out[idx + 2 * width * height] = b;
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
  node->get_parameter("openvins.config", openvins_config, openvins_config);
  node->get_parameter("tensorrt.onnx_path", onnx_path, onnx_path);

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
  const bool depth_ready = depth_net.load_engine(onnx_path, true);
  if (!depth_ready) {
    std::cerr << "DepthNet failed to load; running in placeholder mode." << std::endl;
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

  FrameQueue frame_queue;
  std::mutex imu_mutex;
  ImuSample latest_imu;
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

        {
          std::lock_guard<std::mutex> lock(imu_mutex);
          latest_imu = sample;
        }

        vio.feed_measurement(sample.timestamp, cv::Mat(), sample.accel, sample.gyro);
      });
  (void)imu_sub;

  std::atomic<bool> running(true);

  std::thread imu_thread;
  if (synthetic_imu) {
    imu_thread = std::thread([&]() {
      nature::node::Rate rate(imu_rate_hz);
      while (running.load()) {
        ImuSample sample;
        sample.timestamp = node->get_now_seconds();
        sample.accel = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        sample.gyro = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        {
          std::lock_guard<std::mutex> lock(imu_mutex);
          latest_imu = sample;
        }
        vio.feed_measurement(sample.timestamp, cv::Mat(), sample.accel, sample.gyro);
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
        frame_queue.push(std::move(frame));
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
    rgb_gpu = rgb_cpu.data();
    sparse_gpu = sparse_cpu.data();
  }

  std::thread depth_thread([&]() {
    while (running.load()) {
      Frame frame;
      if (!frame_queue.wait_pop(frame)) {
        break;
      }

      ImuSample imu_sample;
      {
        std::lock_guard<std::mutex> lock(imu_mutex);
        imu_sample = latest_imu;
      }

      vio.feed_measurement(frame.timestamp, frame.image, imu_sample.accel, imu_sample.gyro);

      nature::perception::vio::VIOUpdate vio_update;
      if (!vio.get_latest_update(vio_update)) {
        continue;
      }

      fill_rgb_buffer(frame.image, camera_width, camera_height, rgb_cpu);
#ifdef NATURE_HAS_CUDA
      if (use_gpu) {
        cudaMemcpyAsync(rgb_gpu, rgb_cpu.data(),
                        static_cast<size_t>(camera_width * camera_height * 3) * sizeof(float),
                        cudaMemcpyHostToDevice, stream);
      }
#endif

      nature::perception::depth::generate_sparse_depth_map(
          vio_update, T_bc, K, sparse_gpu, stream);

      nature::perception::depth::DepthResult depth_result;
      bool have_depth = false;
      if (depth_ready && depth_net.infer(rgb_gpu, sparse_gpu, depth_result, stream)) {
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
      }

      auto odom_msg = vio_to_odometry(vio_update, node->get_now_seconds());
      odom_pub->publish(odom_msg);
    }
  });

  while (nature::node::ok()) {
    node->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  running.store(false);
  frame_queue.shutdown();
  if (imu_thread.joinable()) {
    imu_thread.join();
  }
  if (camera_thread.joinable()) {
    camera_thread.join();
  }
  if (depth_thread.joinable()) {
    depth_thread.join();
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
