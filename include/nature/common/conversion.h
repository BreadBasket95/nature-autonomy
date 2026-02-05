//
// Placeholder conversion helpers between internal common types and ICD-style message structs.
//
#ifndef NATURE_CONVERSION_H
#define NATURE_CONVERSION_H

#include <cmath>

#include "nature/common/common_objects.h"
#include "nature/messaging/message_types.h"
#include "nature/nature_utils.h"

namespace nature {
namespace common {

inline common::PointCloud FromMsg(const nature::msg::PointCloud &point_cloud) {
  common::PointCloud pc;
  pc.points.reserve(point_cloud.points.size());
  for (const auto &p : point_cloud.points) {
    pc.points.emplace_back(utils::vec3{p.x, p.y, p.z});
  }
  return pc;
}

inline nature::msg::PointCloud ToMsg(const common::PointCloud &point_cloud) {
  nature::msg::PointCloud pc;
  pc.points.reserve(point_cloud.points.size());
  for (const auto &p : point_cloud.points) {
    nature::msg::Point32 p_insert;
    p_insert.x = p.x;
    p_insert.y = p.y;
    p_insert.z = p.z;
    pc.points.push_back(p_insert);
  }
  return pc;
}

inline common::Twist FromMsg(const nature::msg::Twist &twist) {
  common::Twist converted;
  converted.linear.x = twist.linear.x;
  converted.linear.y = twist.linear.y;
  converted.linear.z = twist.linear.z;
  converted.angular.x = twist.angular.x;
  converted.angular.y = twist.angular.y;
  converted.angular.z = twist.angular.z;
  return converted;
}

inline nature::msg::Twist ToMsg(const common::Twist &twist) {
  nature::msg::Twist converted;
  converted.linear.x = twist.linear.x;
  converted.linear.y = twist.linear.y;
  converted.linear.z = twist.linear.z;
  converted.angular.x = twist.angular.x;
  converted.angular.y = twist.angular.y;
  converted.angular.z = twist.angular.z;
  return converted;
}

inline common::Pose FromMsg(const nature::msg::Pose &pose) {
  common::Pose converted;
  converted.position.x = pose.position.x;
  converted.position.y = pose.position.y;
  converted.position.z = pose.position.z;
  converted.orientation.w = pose.orientation.w;
  converted.orientation.x = pose.orientation.x;
  converted.orientation.y = pose.orientation.y;
  converted.orientation.z = pose.orientation.z;
  return converted;
}

inline nature::msg::Pose ToMsg(const common::Pose &pose) {
  nature::msg::Pose converted;
  converted.position.x = pose.position.x;
  converted.position.y = pose.position.y;
  converted.position.z = pose.position.z;
  converted.orientation.w = pose.orientation.w;
  converted.orientation.x = pose.orientation.x;
  converted.orientation.y = pose.orientation.y;
  converted.orientation.z = pose.orientation.z;
  return converted;
}

inline common::Header FromMsg(const nature::msg::Header &header) {
  common::Header converted;
  converted.frame_id = header.frame_id;
  converted.seconds = header.stamp;
  const double stamp = header.stamp;
  const double sec_floor = std::floor(stamp);
  converted.sec = static_cast<int32_t>(sec_floor);
  const double fractional = stamp - sec_floor;
  converted.nanosec = static_cast<int32_t>(fractional * 1e9);
  return converted;
}

inline nature::msg::Header ToMsg(const common::Header &header) {
  nature::msg::Header converted;
  converted.frame_id = header.frame_id;
  if (header.seconds != 0.0) {
    converted.stamp = header.seconds;
  } else {
    converted.stamp = static_cast<double>(header.sec) + static_cast<double>(header.nanosec) * 1e-9;
  }
  return converted;
}

inline common::PoseStamped FromMsg(const nature::msg::PoseStamped &pose_stamped) {
  common::PoseStamped converted;
  converted.header = FromMsg(pose_stamped.header);
  converted.pose = FromMsg(pose_stamped.pose);
  return converted;
}

inline nature::msg::PoseStamped ToMsg(const common::PoseStamped &pose_stamped) {
  nature::msg::PoseStamped converted;
  converted.header = ToMsg(pose_stamped.header);
  converted.pose = ToMsg(pose_stamped.pose);
  return converted;
}

inline common::Odometry FromMsg(const nature::msg::Odometry &odometry) {
  common::Odometry converted;
  converted.pose = FromMsg(odometry.pose.pose);
  return converted;
}

inline nature::msg::Odometry ToMsg(const common::Odometry &odometry) {
  nature::msg::Odometry converted;
  converted.pose.pose = ToMsg(odometry.pose);
  return converted;
}

inline common::Path FromMsg(const nature::msg::Path &path) {
  common::Path converted;
  converted.poses.reserve(path.poses.size());
  for (const auto &p : path.poses) {
    converted.poses.push_back(FromMsg(p));
  }
  return converted;
}

inline nature::msg::Path ToMsg(const common::Path &path) {
  nature::msg::Path converted;
  converted.poses.reserve(path.poses.size());
  for (const auto &p : path.poses) {
    converted.poses.push_back(ToMsg(p));
  }
  return converted;
}

inline common::OccupancyGrid FromMsg(const nature::msg::OccupancyGrid &occupancy_grid) {
  common::OccupancyGrid converted;
  converted.header = FromMsg(occupancy_grid.header);
  converted.info.resolution = occupancy_grid.info.resolution;
  converted.info.width = occupancy_grid.info.width;
  converted.info.height = occupancy_grid.info.height;
  converted.info.origin.position.x = occupancy_grid.info.origin.position.x;
  converted.info.origin.position.y = occupancy_grid.info.origin.position.y;
  converted.info.origin.position.z = occupancy_grid.info.origin.position.z;
  converted.info.origin.orientation.x = occupancy_grid.info.origin.orientation.x;
  converted.info.origin.orientation.y = occupancy_grid.info.origin.orientation.y;
  converted.info.origin.orientation.z = occupancy_grid.info.origin.orientation.z;
  converted.info.origin.orientation.w = occupancy_grid.info.origin.orientation.w;
  return converted;
}

inline nature::msg::OccupancyGrid ToMsg(const common::OccupancyGrid &occupancy_grid) {
  nature::msg::OccupancyGrid converted;
  converted.header = ToMsg(occupancy_grid.header);
  converted.info.resolution = occupancy_grid.info.resolution;
  converted.info.width = occupancy_grid.info.width;
  converted.info.height = occupancy_grid.info.height;
  converted.info.origin.position.x = occupancy_grid.info.origin.position.x;
  converted.info.origin.position.y = occupancy_grid.info.origin.position.y;
  converted.info.origin.position.z = occupancy_grid.info.origin.position.z;
  converted.info.origin.orientation.x = occupancy_grid.info.origin.orientation.x;
  converted.info.origin.orientation.y = occupancy_grid.info.origin.orientation.y;
  converted.info.origin.orientation.z = occupancy_grid.info.origin.orientation.z;
  converted.info.origin.orientation.w = occupancy_grid.info.origin.orientation.w;
  return converted;
}

} // namespace common
} // namespace nature

#endif // NATURE_CONVERSION_H
