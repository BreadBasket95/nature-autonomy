//
// Placeholder node/runtime helpers for non-middleware messaging.
//
#ifndef NATURE_NODE_PROXY_H
#define NATURE_NODE_PROXY_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <unordered_map>

#include "nature/messaging/message_types.h"

namespace nature {
namespace node {

using Time = double;
using Duration = double;

inline Duration make_duration(float period) {
  return static_cast<double>(period);
}

inline Duration make_duration(int32_t sec, int32_t nsec) {
  return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

template <typename MessageT>
class Publisher {
public:
  explicit Publisher(const std::string &topic_name, int /*qos*/) : topic_(topic_name) {}
  Publisher() = default;

  void publish(const MessageT &msg) {
    last_message_ = std::make_shared<MessageT>(msg);
  }

  std::shared_ptr<const MessageT> last_message() const { return last_message_; }

private:
  std::string topic_;
  std::shared_ptr<MessageT> last_message_;
};

template <typename MessageT, typename CallbackT>
class Subscriber {
public:
  Subscriber(const std::string &topic_name, int /*qos*/, CallbackT &&callback)
      : topic_(topic_name), callback_(std::forward<CallbackT>(callback)) {}

  void receive(const MessageT &msg) {
    if (callback_) {
      callback_(std::make_shared<MessageT>(msg));
    }
  }

private:
  std::string topic_;
  std::function<void(std::shared_ptr<MessageT>)> callback_;
};

inline double seconds_from_header(const nature::msg::Header &header) {
  return header.stamp;
}

inline Time time_from_seconds(double sec) {
  return sec;
}

inline void inc_seq(nature::msg::Header &header) {
  header.seq++;
}

inline void set_seq(nature::msg::Header &header, int seq) {
  header.seq = seq;
}

inline bool ok() {
  return true;
}

inline void init(int /*argc*/, char ** /*argv*/, const std::string & /*node_name*/) {}

class Rate {
public:
  explicit Rate(double hz);
  void sleep();

private:
  std::chrono::duration<double> period_;
};

class NodeProxy {
public:
  explicit NodeProxy(const std::string &node_name);

  template <typename ParameterT>
  bool get_parameter(const std::string &name, ParameterT &parameter_out, const ParameterT default_value) {
    const std::string key = (!name.empty() && name[0] == '~') ? name.substr(1) : name;
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = parameters_.find(key);
    if (it == parameters_.end()) {
      parameter_out = default_value;
      return false;
    }
    auto holder = std::dynamic_pointer_cast<ParameterHolder<ParameterT>>(it->second);
    if (!holder) {
      parameter_out = default_value;
      return false;
    }
    parameter_out = holder->value_;
    return true;
  }

  template <typename ParameterT>
  void set_parameter(const std::string &name, const ParameterT &value) {
    const std::string key = (!name.empty() && name[0] == '~') ? name.substr(1) : name;
    std::lock_guard<std::mutex> lock(mutex_);
    auto storage = std::make_shared<ParameterHolder<ParameterT>>(value);
    parameters_[key] = storage;
  }

  template <typename MessageT>
  std::shared_ptr<Publisher<MessageT>> create_publisher(const std::string &topic_name, int qos) {
    return std::make_shared<Publisher<MessageT>>(topic_name, qos);
  }

  template <typename MessageT, typename CallbackT>
  std::shared_ptr<Subscriber<MessageT, CallbackT>> create_subscription(const std::string &topic_name, int qos,
                                                                       CallbackT &&callback) {
    return std::make_shared<Subscriber<MessageT, CallbackT>>(topic_name, qos, std::forward<CallbackT>(callback));
  }

  Time get_stamp() const;
  double get_now_seconds() const;
  void spin_some();

private:
  struct ParameterBase {
    virtual ~ParameterBase() = default;
  };

  template <typename T>
  struct ParameterHolder : ParameterBase {
    explicit ParameterHolder(const T &value) : value_(value) {}
    T value_;
  };

  std::string name_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<ParameterBase>> parameters_;
  std::chrono::steady_clock::time_point start_time_;
};

inline std::shared_ptr<NodeProxy> make_shared(const std::string &name) {
  return std::make_shared<NodeProxy>(name);
}

inline std::shared_ptr<NodeProxy> init_node(int argc, char *argv[], const std::string &name) {
  init(argc, argv, name);
  return make_shared(name);
}

} // namespace node
} // namespace nature

#endif // NATURE_NODE_PROXY_H
