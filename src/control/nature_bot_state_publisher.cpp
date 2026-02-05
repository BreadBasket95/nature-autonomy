// messaging includes
#include "nature/messaging/message_types.h"
#include "nature/node/node_proxy.h"

#include <iostream>

nature::msg::Odometry latest_odom;
bool odom_rcvd = false;

/**
 * @brief Handle incoming odometry updates.
 * @param rcv_odom Incoming odometry message.
 * @details Stores the latest odometry for publishing as a PoseStamped.
 */
void OdometryCallback(nature::msg::OdometryPtr rcv_odom) {
  latest_odom = *rcv_odom;
  odom_rcvd = true;
}

/**
 * @brief Entry point for the state publisher node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit code.
 * @details Subscribes to odometry and republishes the pose component for
 *          consumers that only need pose data.
 */
int main(int argc, char **argv) {
  auto n = nature::node::init_node(argc, argv, "nature_state_publisher");

  auto odom_sub = n->create_subscription<nature::msg::Odometry>("nature/odometry", 100, OdometryCallback);
  auto pose_pub = n->create_publisher<nature::msg::PoseStamped>("nature/state_pose", 10);

  nature::node::Rate loop_rate(100.0);
  while (nature::node::ok()) {
    if (odom_rcvd) {
      nature::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = n->get_stamp();
      pose_msg.header.frame_id = "odom";
      pose_msg.pose = latest_odom.pose.pose;
      pose_pub->publish(pose_msg);
    }
    n->spin_some();
    loop_rate.sleep();
  }

  return 0;
}
