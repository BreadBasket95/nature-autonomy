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

/**
 * @brief Convert a placeholder PointCloud message to an internal common PointCloud.
 * @param point_cloud Incoming message-style point cloud.
 * @return Common PointCloud with points copied into utils::vec3 format.
 * @details Iterates over all points, translating float components into the internal
 *          math struct. Used by perception and mapping stages to work on shared types.
 */
inline common::PointCloud FromMsg(const nature::msg::PointCloud &point_cloud) {
  common::PointCloud pc;
  pc.points.reserve(point_cloud.points.size());
  for (const auto &p : point_cloud.points) {
    pc.points.emplace_back(utils::vec3{p.x, p.y, p.z});
  }
  return pc;
}

/**
 * @brief Convert an internal common PointCloud to a placeholder PointCloud message.
 * @param point_cloud Internal point cloud with utils::vec3 points.
 * @return Message-style PointCloud with Point32 entries.
 * @details Copies each point component into the message struct; no frame metadata
 *          is set here and must be filled by the caller.
 */
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

/**
 * @brief Convert a placeholder Twist message to an internal common Twist.
 * @param twist Incoming message-style twist.
 * @return Common Twist with linear and angular vectors copied.
 * @details Used by controllers and simulators that operate on common::Twist.
 */
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

/**
 * @brief Convert an internal common Twist to a placeholder Twist message.
 * @param twist Internal Twist to convert.
 * @return Message-style Twist with copied components.
 * @details Enables publishing common::Twist over the placeholder messaging layer.
 */
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

/**
 * @brief Convert a placeholder Pose message to an internal common Pose.
 * @param pose Incoming message-style pose.
 * @return Common Pose with position and orientation copied.
 * @details Used to bridge perception/planning data into internal math structs.
 */
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

/**
 * @brief Convert an internal common Pose to a placeholder Pose message.
 * @param pose Internal pose to convert.
 * @return Message-style Pose with copied fields.
 * @details Provides a message representation for publishing or logging.
 */
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

/**
 * @brief Convert a placeholder Header to a common Header.
 * @param header Incoming message-style header.
 * @return Common Header with frame ID and time copied.
 * @details Converts seconds into sec/nanosec fields for legacy internal usage.
 */
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

/**
 * @brief Convert a common Header to a placeholder Header.
 * @param header Internal header to convert.
 * @return Message-style Header with frame ID and stamp populated.
 * @details Uses header.seconds when available; otherwise builds from sec/nanosec.
 */
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

/**
 * @brief Convert a placeholder PoseStamped to a common PoseStamped.
 * @param pose_stamped Incoming message-style pose with header.
 * @return Common PoseStamped with header and pose converted.
 * @details Combines header and pose conversions for pipeline convenience.
 */
inline common::PoseStamped FromMsg(const nature::msg::PoseStamped &pose_stamped) {
  common::PoseStamped converted;
  converted.header = FromMsg(pose_stamped.header);
  converted.pose = FromMsg(pose_stamped.pose);
  return converted;
}

/**
 * @brief Convert a common PoseStamped to a placeholder PoseStamped.
 * @param pose_stamped Internal pose with header.
 * @return Message-style PoseStamped with converted fields.
 * @details Used when publishing waypoints or pose arrays.
 */
inline nature::msg::PoseStamped ToMsg(const common::PoseStamped &pose_stamped) {
  nature::msg::PoseStamped converted;
  converted.header = ToMsg(pose_stamped.header);
  converted.pose = ToMsg(pose_stamped.pose);
  return converted;
}

/**
 * @brief Convert a placeholder Odometry message to a common Odometry.
 * @param odometry Incoming message-style odometry.
 * @return Common Odometry with pose converted.
 * @details Only pose is carried over in current internal representation.
 */
inline common::Odometry FromMsg(const nature::msg::Odometry &odometry) {
  common::Odometry converted;
  converted.pose = FromMsg(odometry.pose.pose);
  return converted;
}

/**
 * @brief Convert a common Odometry to a placeholder Odometry message.
 * @param odometry Internal odometry.
 * @return Message-style Odometry with pose populated.
 * @details Leaves header unset; caller should set timestamps/frame IDs.
 */
inline nature::msg::Odometry ToMsg(const common::Odometry &odometry) {
  nature::msg::Odometry converted;
  converted.pose.pose = ToMsg(odometry.pose);
  return converted;
}

/**
 * @brief Convert a placeholder Path message to a common Path.
 * @param path Incoming message-style path.
 * @return Common Path with converted pose list.
 * @details Iterates over pose stamps and converts each to the internal type.
 */
inline common::Path FromMsg(const nature::msg::Path &path) {
  common::Path converted;
  converted.poses.reserve(path.poses.size());
  for (const auto &p : path.poses) {
    converted.poses.push_back(FromMsg(p));
  }
  return converted;
}

/**
 * @brief Convert a common Path to a placeholder Path message.
 * @param path Internal path to convert.
 * @return Message-style Path with pose stamps copied.
 * @details Used by planners to publish global and local trajectories.
 */
inline nature::msg::Path ToMsg(const common::Path &path) {
  nature::msg::Path converted;
  converted.poses.reserve(path.poses.size());
  for (const auto &p : path.poses) {
    converted.poses.push_back(ToMsg(p));
  }
  return converted;
}

/**
 * @brief Convert a placeholder OccupancyGrid to a common OccupancyGrid.
 * @param occupancy_grid Incoming message-style grid.
 * @return Common OccupancyGrid with header and metadata copied.
 * @details Converts the grid origin and size fields used by planning.
 */
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

/**
 * @brief Convert a common OccupancyGrid to a placeholder OccupancyGrid.
 * @param occupancy_grid Internal grid to convert.
 * @return Message-style OccupancyGrid with metadata and header populated.
 * @details The grid data vector is not copied here because the common type
 *          currently only stores metadata.
 */
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
