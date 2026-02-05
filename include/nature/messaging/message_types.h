//
// Messaging type aliases and pointer helpers for placeholder ICD structs.
//
#ifndef NATURE_MESSAGE_TYPES_H
#define NATURE_MESSAGE_TYPES_H

#include <memory>
#include "nature/messaging/placeholder_messages.h"

namespace nature {
namespace msg {

using PointCloudPtr = std::shared_ptr<PointCloud>;
using PointCloud2Ptr = std::shared_ptr<PointCloud2>;
using NavSatFixPtr = std::shared_ptr<NavSatFix>;
using PointFieldPtr = std::shared_ptr<PointField>;
using JointStatePtr = std::shared_ptr<JointState>;
using TwistPtr = std::shared_ptr<Twist>;
using Point32Ptr = std::shared_ptr<Point32>;
using QuaternionPtr = std::shared_ptr<Quaternion>;
using PointPtr = std::shared_ptr<Point>;
using PoseStampedPtr = std::shared_ptr<PoseStamped>;
using PointStampedPtr = std::shared_ptr<PointStamped>;
using OccupancyGridPtr = std::shared_ptr<OccupancyGrid>;
using PathPtr = std::shared_ptr<Path>;
using OdometryPtr = std::shared_ptr<Odometry>;
using ImuPtr = std::shared_ptr<Imu>;
using ImagePtr = std::shared_ptr<Image>;
using MarkerPtr = std::shared_ptr<Marker>;
using MarkerArrayPtr = std::shared_ptr<MarkerArray>;
using Float64Ptr = std::shared_ptr<Float64>;
using Float64MultiArrayPtr = std::shared_ptr<Float64MultiArray>;
using Int32Ptr = std::shared_ptr<Int32>;
using ClockPtr = std::shared_ptr<Clock>;

} // namespace msg
} // namespace nature

#endif // NATURE_MESSAGE_TYPES_H
