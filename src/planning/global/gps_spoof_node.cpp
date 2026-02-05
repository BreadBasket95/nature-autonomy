/**
 * \file gps_spoof_node.cpp
 *
 * Placeholder node that publishes a fixed GPS fix at a constant rate.
 *
 * \author Chris Goodin
 *
 * \contact dwc2@cavs.msstate.edu
 *
 * \date 1/21/2022
 */

// messaging includes
#include "nature/messaging/message_types.h"
#include "nature/node/node_proxy.h"

/**
 * @brief Entry point for the GPS spoofing node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit code.
 * @details Publishes a fixed NavSatFix at a constant rate for testing
 *          downstream GPS/ENU conversion and planning nodes.
 */
int main(int argc, char **argv) {
  auto n = nature::node::init_node(argc, argv, "gps_spoof_node");

  auto navsat_pub = n->create_publisher<nature::msg::NavSatFix>("nature/enu_waypoints", 10);

  nature::msg::NavSatFix fix;
  fix.latitude = 33.47045;
  fix.longitude = -88.78649;
  fix.altitude = 86.0;

  nature::node::Rate loop_rate(10.0);
  while (nature::node::ok()) {
    fix.header.stamp = n->get_stamp();
    navsat_pub->publish(fix);
    n->spin_some();
    loop_rate.sleep();
  }

  return 0;
}
