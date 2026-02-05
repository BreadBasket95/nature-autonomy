//
// Placeholder clock publisher for non-middleware messaging.
//
#ifndef NATURE_CLOCK_PUBLISHER_H
#define NATURE_CLOCK_PUBLISHER_H

#include <memory>

#include "nature/messaging/message_types.h"
#include "nature/node/node_proxy.h"

namespace nature {
namespace node {

class ClockPublisher {
public:
  /**
   * @brief Create a shared ClockPublisher instance.
   * @param topic_name Topic name to publish clock updates on.
   * @param qos Placeholder QoS depth (unused).
   * @param node NodeProxy used to create the publisher.
   * @return Shared pointer to a new ClockPublisher.
   * @details Factory helper to mirror ROS-style make_shared usage in nodes.
   */
  static std::shared_ptr<ClockPublisher> make_shared(const std::string &topic_name, int qos,
                                                     std::shared_ptr<NodeProxy> node);
  /**
   * @brief Construct a clock publisher.
   * @param topic_name Topic name for clock messages.
   * @param qos Placeholder QoS depth (unused).
   * @param node NodeProxy used to create the publisher.
   * @details Initializes the underlying publisher for the placeholder messaging layer.
   */
  ClockPublisher(const std::string &topic_name, int qos, std::shared_ptr<NodeProxy> node);
  /**
   * @brief Publish a clock tick.
   * @param elapsed_time Elapsed simulation time in seconds.
   * @details Populates a Clock message and publishes it; used by simulation nodes
   *          when running in simulated time mode.
   */
  void publish(double elapsed_time);

private:
  std::shared_ptr<nature::node::Publisher<nature::msg::Clock>> pub_;
};

} // namespace node
} // namespace nature

#endif // NATURE_CLOCK_PUBLISHER_H
