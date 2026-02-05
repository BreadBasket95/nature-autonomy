#include "nature/node/clock_publisher.h"

namespace nature {
namespace node {

std::shared_ptr<ClockPublisher> ClockPublisher::make_shared(const std::string &topic_name, int qos,
                                                            std::shared_ptr<NodeProxy> node) {
  return std::make_shared<ClockPublisher>(topic_name, qos, node);
}

ClockPublisher::ClockPublisher(const std::string &topic_name, int qos, std::shared_ptr<NodeProxy> node) {
  pub_ = node->create_publisher<nature::msg::Clock>(topic_name, qos);
}

void ClockPublisher::publish(double elapsed_time) {
  if (!pub_) {
    return;
  }
  nature::msg::Clock clock_msg;
  clock_msg.clock = elapsed_time;
  pub_->publish(clock_msg);
}

} // namespace node
} // namespace nature
