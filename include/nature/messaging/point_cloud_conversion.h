//
// Minimal point cloud conversion helpers for placeholder message structs.
//
#ifndef NATURE_POINT_CLOUD_CONVERSION_H
#define NATURE_POINT_CLOUD_CONVERSION_H

#include <cstring>
#include <vector>

#include "nature/messaging/message_types.h"

namespace nature {
namespace messaging {

/**
 * @brief Convert a dense PointCloud2 buffer into a PointCloud list of points.
 * @param in_cloud Incoming PointCloud2 with packed binary data.
 * @param out_cloud Output PointCloud populated with Point32 points and optional channels.
 * @return True if conversion succeeded; false if required fields are missing or input is empty.
 * @details Locates x/y/z (and optional segmentation) fields, then memcpy's each
 *          point into a Point32 list. This is a temporary bridge while the stack
 *          migrates off ROS message types and into ICD-style structs.
 */
inline bool convertPointCloud2ToPointCloud(const nature::msg::PointCloud2 &in_cloud,
                                           nature::msg::PointCloud &out_cloud) {
  out_cloud.header = in_cloud.header;
  out_cloud.points.clear();
  out_cloud.channels.clear();

  if (in_cloud.point_step == 0 || in_cloud.data.empty()) {
    return false;
  }

  int32_t x_offset = -1;
  int32_t y_offset = -1;
  int32_t z_offset = -1;
  int32_t seg_offset = -1;

  for (const auto &field : in_cloud.fields) {
    if (field.name == "x") x_offset = static_cast<int32_t>(field.offset);
    if (field.name == "y") y_offset = static_cast<int32_t>(field.offset);
    if (field.name == "z") z_offset = static_cast<int32_t>(field.offset);
    if (field.name == "segmentation") seg_offset = static_cast<int32_t>(field.offset);
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    return false;
  }

  const size_t point_count = in_cloud.width * in_cloud.height;
  out_cloud.points.reserve(point_count);

  nature::msg::PointCloud::Channel seg_channel;
  if (seg_offset >= 0) {
    seg_channel.name = "segmentation";
    seg_channel.values.reserve(point_count);
  }

  for (size_t idx = 0; idx < point_count; ++idx) {
    const size_t base = idx * in_cloud.point_step;
    if (base + in_cloud.point_step > in_cloud.data.size()) {
      break;
    }
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::memcpy(&x, &in_cloud.data[base + x_offset], sizeof(float));
    std::memcpy(&y, &in_cloud.data[base + y_offset], sizeof(float));
    std::memcpy(&z, &in_cloud.data[base + z_offset], sizeof(float));
    nature::msg::Point32 pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    out_cloud.points.push_back(pt);

    if (seg_offset >= 0) {
      float seg = 0.0f;
      std::memcpy(&seg, &in_cloud.data[base + seg_offset], sizeof(float));
      seg_channel.values.push_back(seg);
    }
  }

  if (seg_offset >= 0) {
    out_cloud.channels.push_back(seg_channel);
  }
  return true;
}

} // namespace messaging
} // namespace nature

#endif // NATURE_POINT_CLOUD_CONVERSION_H
