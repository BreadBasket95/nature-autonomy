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
  static std::shared_ptr<ClockPublisher> make_shared(const std::string &topic_name, int qos,
                                                     std::shared_ptr<NodeProxy> node);
  ClockPublisher(const std::string &topic_name, int qos, std::shared_ptr<NodeProxy> node);
  void publish(double elapsed_time);

private:
  std::shared_ptr<nature::node::Publisher<nature::msg::Clock>> pub_;
};

} // namespace node
} // namespace nature

#endif // NATURE_CLOCK_PUBLISHER_H
