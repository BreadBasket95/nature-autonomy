#include "nature/node/node_proxy.h"

namespace nature {
namespace node {

Rate::Rate(double hz) {
  if (hz <= 0.0) {
    period_ = std::chrono::duration<double>(0.0);
  } else {
    period_ = std::chrono::duration<double>(1.0 / hz);
  }
}

void Rate::sleep() {
  if (period_.count() <= 0.0) {
    return;
  }
  std::this_thread::sleep_for(period_);
}

NodeProxy::NodeProxy(const std::string &node_name) : name_(node_name), start_time_(std::chrono::steady_clock::now()) {}

Time NodeProxy::get_stamp() const {
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - start_time_).count();
}

double NodeProxy::get_now_seconds() const {
  return get_stamp();
}

void NodeProxy::spin_some() {
  // Placeholder: no background event loop yet.
}

} // namespace node
} // namespace nature
