//
// ZMQ transport layer for ICD message exchange.
//
#ifndef NATURE_ZMQ_TRANSPORT_H
#define NATURE_ZMQ_TRANSPORT_H

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace nature {
namespace transport {

class ZmqTransport {
public:
  using Callback = std::function<void(const std::vector<uint8_t> &)>;

  /**
   * @brief Construct a ZMQ transport with PUB/SUB sockets.
   * @param pub_endpoint Endpoint to bind the publisher (e.g., tcp://*:5556).
   * @param sub_endpoint Endpoint to connect the subscriber (e.g., tcp://localhost:5557).
   */
  ZmqTransport(const std::string &pub_endpoint, const std::string &sub_endpoint);

  /**
   * @brief Destroy the transport and close sockets.
   */
  ~ZmqTransport();

  /**
   * @brief Publish a payload on the given topic.
   * @param topic Topic name.
   * @param payload Serialized message payload.
   */
  void publish(const std::string &topic, const std::vector<uint8_t> &payload);

  /**
   * @brief Subscribe to a topic.
   * @param topic Topic name.
   * @param callback Callback invoked with payload bytes.
   */
  void subscribe(const std::string &topic, Callback callback);

  /**
   * @brief Poll for inbound messages once.
   * @param timeout_ms Poll timeout in milliseconds.
   */
  void poll_once(int timeout_ms);

private:
  void *context_ = nullptr;
  void *pub_socket_ = nullptr;
  void *sub_socket_ = nullptr;
  std::unordered_map<std::string, Callback> callbacks_;
};

} // namespace transport
} // namespace nature

#endif // NATURE_ZMQ_TRANSPORT_H
