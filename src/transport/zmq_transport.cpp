//
// ZMQ transport implementation.
//
#include "nature/transport/zmq_transport.h"

#include <cstring>
#include <iostream>

#include <zmq.h>

namespace nature {
namespace transport {

ZmqTransport::ZmqTransport(const std::string &pub_endpoint, const std::string &sub_endpoint) {
  context_ = zmq_ctx_new();
  if (!context_) {
    std::cerr << "Failed to create ZMQ context." << std::endl;
    return;
  }

  pub_socket_ = zmq_socket(context_, ZMQ_PUB);
  sub_socket_ = zmq_socket(context_, ZMQ_SUB);

  if (pub_socket_) {
    if (zmq_bind(pub_socket_, pub_endpoint.c_str()) != 0) {
      std::cerr << "Failed to bind PUB socket: " << pub_endpoint << std::endl;
    }
  }

  if (sub_socket_) {
    if (zmq_connect(sub_socket_, sub_endpoint.c_str()) != 0) {
      std::cerr << "Failed to connect SUB socket: " << sub_endpoint << std::endl;
    }
  }

  // TODO: Configure HWM, linger, and socket options for production.
}

ZmqTransport::~ZmqTransport() {
  if (pub_socket_) {
    zmq_close(pub_socket_);
  }
  if (sub_socket_) {
    zmq_close(sub_socket_);
  }
  if (context_) {
    zmq_ctx_term(context_);
  }
}

void ZmqTransport::publish(const std::string &topic, const std::vector<uint8_t> &payload) {
  if (!pub_socket_) {
    return;
  }
  zmq_msg_t topic_msg;
  zmq_msg_init_size(&topic_msg, topic.size());
  std::memcpy(zmq_msg_data(&topic_msg), topic.data(), topic.size());
  zmq_msg_send(&topic_msg, pub_socket_, ZMQ_SNDMORE);
  zmq_msg_close(&topic_msg);

  zmq_msg_t payload_msg;
  zmq_msg_init_size(&payload_msg, payload.size());
  if (!payload.empty()) {
    std::memcpy(zmq_msg_data(&payload_msg), payload.data(), payload.size());
  }
  zmq_msg_send(&payload_msg, pub_socket_, 0);
  zmq_msg_close(&payload_msg);
}

void ZmqTransport::subscribe(const std::string &topic, Callback callback) {
  if (!sub_socket_) {
    return;
  }
  zmq_setsockopt(sub_socket_, ZMQ_SUBSCRIBE, topic.data(), topic.size());
  callbacks_[topic] = std::move(callback);
}

void ZmqTransport::poll_once(int timeout_ms) {
  if (!sub_socket_) {
    return;
  }
  zmq_pollitem_t items[] = {
      {sub_socket_, 0, ZMQ_POLLIN, 0},
  };
  const int rc = zmq_poll(items, 1, timeout_ms);
  if (rc <= 0) {
    return;
  }
  if (items[0].revents & ZMQ_POLLIN) {
    zmq_msg_t topic_msg;
    zmq_msg_init(&topic_msg);
    if (zmq_msg_recv(&topic_msg, sub_socket_, 0) < 0) {
      zmq_msg_close(&topic_msg);
      return;
    }
    const std::string topic(static_cast<char *>(zmq_msg_data(&topic_msg)),
                            zmq_msg_size(&topic_msg));
    zmq_msg_close(&topic_msg);

    zmq_msg_t payload_msg;
    zmq_msg_init(&payload_msg);
    if (zmq_msg_recv(&payload_msg, sub_socket_, 0) < 0) {
      zmq_msg_close(&payload_msg);
      return;
    }
    const uint8_t *data = static_cast<uint8_t *>(zmq_msg_data(&payload_msg));
    const size_t size = zmq_msg_size(&payload_msg);
    std::vector<uint8_t> payload(data, data + size);
    zmq_msg_close(&payload_msg);

    auto it = callbacks_.find(topic);
    if (it != callbacks_.end()) {
      it->second(payload);
    }
  }
}

} // namespace transport
} // namespace nature
