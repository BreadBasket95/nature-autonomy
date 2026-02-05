//
// Conversion utilities between placeholder messages and ICD structs.
//
#ifndef NATURE_ICD_ADAPTER_H
#define NATURE_ICD_ADAPTER_H

#include "nature/messaging/icd_messages.h"
#include "nature/messaging/placeholder_messages.h"

namespace nature {
namespace transport {

inline nature::icd::Header to_icd_header(const nature::msg::Header &header) {
  nature::icd::Header out;
  out.seq = static_cast<uint32_t>(header.seq);
  out.stamp_ns = static_cast<uint64_t>(header.stamp * 1e9);
  out.frame_id = header.frame_id;
  return out;
}

inline nature::msg::Header to_msg_header(const nature::icd::Header &header) {
  nature::msg::Header out;
  out.seq = static_cast<int32_t>(header.seq);
  out.stamp = static_cast<double>(header.stamp_ns) * 1e-9;
  out.frame_id = header.frame_id;
  return out;
}

inline nature::icd::Imu to_icd(const nature::msg::Imu &imu) {
  nature::icd::Imu out;
  out.header = to_icd_header(imu.header);
  out.accel[0] = imu.linear_acceleration.x;
  out.accel[1] = imu.linear_acceleration.y;
  out.accel[2] = imu.linear_acceleration.z;
  out.gyro[0] = imu.angular_velocity.x;
  out.gyro[1] = imu.angular_velocity.y;
  out.gyro[2] = imu.angular_velocity.z;
  return out;
}

inline nature::msg::Imu to_msg(const nature::icd::Imu &imu) {
  nature::msg::Imu out;
  out.header = to_msg_header(imu.header);
  out.linear_acceleration.x = imu.accel[0];
  out.linear_acceleration.y = imu.accel[1];
  out.linear_acceleration.z = imu.accel[2];
  out.angular_velocity.x = imu.gyro[0];
  out.angular_velocity.y = imu.gyro[1];
  out.angular_velocity.z = imu.gyro[2];
  return out;
}

inline nature::icd::Image to_icd(const nature::msg::Image &image) {
  nature::icd::Image out;
  out.header = to_icd_header(image.header);
  out.width = image.width;
  out.height = image.height;
  out.encoding = image.encoding;
  out.step = image.step;
  out.data = image.data;
  return out;
}

inline nature::msg::Image to_msg(const nature::icd::Image &image) {
  nature::msg::Image out;
  out.header = to_msg_header(image.header);
  out.width = image.width;
  out.height = image.height;
  out.encoding = image.encoding;
  out.step = image.step;
  out.data = image.data;
  return out;
}

inline nature::icd::Odometry to_icd(const nature::msg::Odometry &odom) {
  nature::icd::Odometry out;
  out.header = to_icd_header(odom.header);
  out.position[0] = odom.pose.pose.position.x;
  out.position[1] = odom.pose.pose.position.y;
  out.position[2] = odom.pose.pose.position.z;
  out.orientation[0] = odom.pose.pose.orientation.x;
  out.orientation[1] = odom.pose.pose.orientation.y;
  out.orientation[2] = odom.pose.pose.orientation.z;
  out.orientation[3] = odom.pose.pose.orientation.w;
  out.linear_vel[0] = odom.twist.twist.linear.x;
  out.linear_vel[1] = odom.twist.twist.linear.y;
  out.linear_vel[2] = odom.twist.twist.linear.z;
  out.angular_vel[0] = odom.twist.twist.angular.x;
  out.angular_vel[1] = odom.twist.twist.angular.y;
  out.angular_vel[2] = odom.twist.twist.angular.z;
  return out;
}

inline nature::msg::Odometry to_msg(const nature::icd::Odometry &odom) {
  nature::msg::Odometry out;
  out.header = to_msg_header(odom.header);
  out.pose.pose.position.x = odom.position[0];
  out.pose.pose.position.y = odom.position[1];
  out.pose.pose.position.z = odom.position[2];
  out.pose.pose.orientation.x = odom.orientation[0];
  out.pose.pose.orientation.y = odom.orientation[1];
  out.pose.pose.orientation.z = odom.orientation[2];
  out.pose.pose.orientation.w = odom.orientation[3];
  out.twist.twist.linear.x = odom.linear_vel[0];
  out.twist.twist.linear.y = odom.linear_vel[1];
  out.twist.twist.linear.z = odom.linear_vel[2];
  out.twist.twist.angular.x = odom.angular_vel[0];
  out.twist.twist.angular.y = odom.angular_vel[1];
  out.twist.twist.angular.z = odom.angular_vel[2];
  return out;
}

inline nature::icd::OccupancyGrid to_icd(const nature::msg::OccupancyGrid &grid) {
  nature::icd::OccupancyGrid out;
  out.header = to_icd_header(grid.header);
  out.resolution = grid.info.resolution;
  out.width = grid.info.width;
  out.height = grid.info.height;
  out.origin_x = static_cast<float>(grid.info.origin.position.x);
  out.origin_y = static_cast<float>(grid.info.origin.position.y);
  out.data = grid.data;
  return out;
}

inline nature::msg::OccupancyGrid to_msg(const nature::icd::OccupancyGrid &grid) {
  nature::msg::OccupancyGrid out;
  out.header = to_msg_header(grid.header);
  out.info.resolution = grid.resolution;
  out.info.width = grid.width;
  out.info.height = grid.height;
  out.info.origin.position.x = grid.origin_x;
  out.info.origin.position.y = grid.origin_y;
  out.data = grid.data;
  return out;
}

} // namespace transport
} // namespace nature

#endif // NATURE_ICD_ADAPTER_H
