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

/**
 * @brief Create a duration in seconds from a float period.
 * @param period Duration in seconds.
 * @return Duration expressed as a double.
 * @details Convenience helper to mirror ROS-style duration creation while
 *          running on the placeholder runtime.
 */
inline Duration make_duration(float period) {
  return static_cast<double>(period);
}

/**
 * @brief Create a duration from seconds and nanoseconds.
 * @param sec Whole seconds component.
 * @param nsec Nanoseconds component.
 * @return Duration expressed as a double in seconds.
 * @details Used to mirror ROS duration APIs in the refactored stack.
 */
inline Duration make_duration(int32_t sec, int32_t nsec) {
  return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

template <typename MessageT>
class Publisher {
public:
  /**
   * @brief Construct a publisher for a topic.
   * @param topic_name Topic name to associate with this publisher.
   * @param qos Placeholder QoS depth (unused).
   * @details Stores the topic name for bookkeeping; no transport is created yet.
   */
  explicit Publisher(const std::string &topic_name, int /*qos*/) : topic_(topic_name) {}
  /**
   * @brief Default-construct a publisher.
   * @details Leaves the topic name empty; used when a publisher is declared
   *          before being assigned by a NodeProxy.
   */
  Publisher() = default;

  /**
   * @brief Publish a message on the topic.
   * @param msg Message to publish.
   * @details In the placeholder runtime, this stores the last message so tests
   *          or adapters can access it. The ZMQ transport will replace this.
   */
  void publish(const MessageT &msg) {
    last_message_ = std::make_shared<MessageT>(msg);
  }

  /**
   * @brief Access the last message published.
   * @return Shared pointer to the most recent message, or null if none.
   * @details Useful for unit tests and temporary integration hooks.
   */
  std::shared_ptr<const MessageT> last_message() const { return last_message_; }

private:
  std::string topic_;
  std::shared_ptr<MessageT> last_message_;
};

template <typename MessageT, typename CallbackT>
class Subscriber {
public:
  /**
   * @brief Construct a subscriber for a topic.
   * @param topic_name Topic name to associate with this subscriber.
   * @param qos Placeholder QoS depth (unused).
   * @param callback Callback invoked when a message is received.
   * @details Stores the callback and topic so adapters can simulate delivery.
   */
  Subscriber(const std::string &topic_name, int /*qos*/, CallbackT &&callback)
      : topic_(topic_name), callback_(std::forward<CallbackT>(callback)) {}

  /**
   * @brief Deliver a message to the subscriber.
   * @param msg Message instance to forward.
   * @details Wraps the message in a shared_ptr to match ROS-style callbacks.
   */
  void receive(const MessageT &msg) {
    if (callback_) {
      callback_(std::make_shared<MessageT>(msg));
    }
  }

private:
  std::string topic_;
  std::function<void(std::shared_ptr<MessageT>)> callback_;
};

/**
 * @brief Extract time in seconds from a message header.
 * @param header Header containing a double-precision stamp.
 * @return Timestamp in seconds.
 * @details Mirrors ROS Time conversions to keep call sites unchanged.
 */
inline double seconds_from_header(const nature::msg::Header &header) {
  return header.stamp;
}

/**
 * @brief Wrap a seconds value as a Time.
 * @param sec Time in seconds.
 * @return Time value.
 * @details Placeholder helper used where ROS Time was previously expected.
 */
inline Time time_from_seconds(double sec) {
  return sec;
}

/**
 * @brief Increment the sequence number in a header.
 * @param header Header to mutate.
 * @details Used by simulation nodes that previously relied on ROS seq fields.
 */
inline void inc_seq(nature::msg::Header &header) {
  header.seq++;
}

/**
 * @brief Set the sequence number in a header.
 * @param header Header to mutate.
 * @param seq Sequence value to assign.
 * @details Keeps compatibility with legacy logging/visualization expectations.
 */
inline void set_seq(nature::msg::Header &header, int seq) {
  header.seq = seq;
}

/**
 * @brief Indicate whether the runtime should keep spinning.
 * @return True while the placeholder runtime is healthy.
 * @details Currently always returns true; replace when ZMQ runtime adds shutdown.
 */
inline bool ok() {
  return true;
}

/**
 * @brief Initialize the placeholder runtime.
 * @param argc Process argc.
 * @param argv Process argv.
 * @param node_name Name of the node being initialized.
 * @details No-op placeholder that mirrors ROS initialization signatures.
 */
inline void init(int /*argc*/, char ** /*argv*/, const std::string & /*node_name*/) {}

class Rate {
public:
  /**
   * @brief Construct a rate controller from a frequency.
   * @param hz Frequency in Hz.
   * @details Computes the sleep period as 1/hz for loop throttling.
   */
  explicit Rate(double hz);
  /**
   * @brief Sleep to maintain the requested rate.
   * @details Uses std::this_thread::sleep_for with the precomputed period.
   */
  void sleep();

private:
  std::chrono::duration<double> period_;
};

class NodeProxy {
public:
  /**
   * @brief Construct a placeholder node proxy.
   * @param node_name Name of the node.
   * @details Starts a steady clock reference for timestamp generation.
   */
  explicit NodeProxy(const std::string &node_name);

  template <typename ParameterT>
  /**
   * @brief Retrieve a parameter value by name.
   * @param name Parameter name, optionally prefixed with '~'.
   * @param parameter_out Output reference for the value.
   * @param default_value Fallback value if the parameter is unset or type-mismatched.
   * @return True if the parameter was found and type-matched; false otherwise.
   * @details Mimics ROS parameter APIs; stores parameters in an in-memory map.
   */
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
  /**
   * @brief Set a parameter value by name.
   * @param name Parameter name, optionally prefixed with '~'.
   * @param value Value to store.
   * @details Stores parameters in an in-memory map for placeholder access.
   */
  void set_parameter(const std::string &name, const ParameterT &value) {
    const std::string key = (!name.empty() && name[0] == '~') ? name.substr(1) : name;
    std::lock_guard<std::mutex> lock(mutex_);
    auto storage = std::make_shared<ParameterHolder<ParameterT>>(value);
    parameters_[key] = storage;
  }

  template <typename MessageT>
  /**
   * @brief Create a publisher for a given topic.
   * @param topic_name Topic to publish on.
   * @param qos Placeholder QoS depth (unused).
   * @return Shared pointer to a Publisher instance.
   * @details Used by nodes to publish placeholder messages before ZMQ transport lands.
   */
  std::shared_ptr<Publisher<MessageT>> create_publisher(const std::string &topic_name, int qos) {
    return std::make_shared<Publisher<MessageT>>(topic_name, qos);
  }

  template <typename MessageT, typename CallbackT>
  /**
   * @brief Create a subscription for a given topic.
   * @param topic_name Topic to subscribe to.
   * @param qos Placeholder QoS depth (unused).
   * @param callback Callback invoked when a message is received.
   * @return Shared pointer to a Subscriber instance.
   * @details Wiring for placeholder subscriptions; actual transport is added later.
   */
  std::shared_ptr<Subscriber<MessageT, CallbackT>> create_subscription(const std::string &topic_name, int qos,
                                                                       CallbackT &&callback) {
    return std::make_shared<Subscriber<MessageT, CallbackT>>(topic_name, qos, std::forward<CallbackT>(callback));
  }

  /**
   * @brief Get the current time stamp in seconds.
   * @return Time value in seconds since node start.
   * @details Uses a steady clock baseline to mimic ROS time progression.
   */
  Time get_stamp() const;
  /**
   * @brief Get the current time in seconds (double).
   * @return Seconds since node start.
   * @details Convenience wrapper used by some legacy nodes.
   */
  double get_now_seconds() const;
  /**
   * @brief Process pending callbacks.
   * @details Placeholder no-op; in a ZMQ runtime this would pump inbound messages.
   */
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

/**
 * @brief Create a shared NodeProxy with the given name.
 * @param name Node name.
 * @return Shared pointer to a new NodeProxy.
 * @details Convenience wrapper to mirror std::make_shared usage across the stack.
 */
inline std::shared_ptr<NodeProxy> make_shared(const std::string &name) {
  return std::make_shared<NodeProxy>(name);
}

/**
 * @brief Initialize the runtime and create a NodeProxy.
 * @param argc Process argc.
 * @param argv Process argv.
 * @param name Node name.
 * @return Shared pointer to the created NodeProxy.
 * @details Mirrors ROS init+node creation in one call for legacy code.
 */
inline std::shared_ptr<NodeProxy> init_node(int argc, char *argv[], const std::string &name) {
  init(argc, argv, name);
  return make_shared(name);
}

} // namespace node
} // namespace nature

#endif // NATURE_NODE_PROXY_H
