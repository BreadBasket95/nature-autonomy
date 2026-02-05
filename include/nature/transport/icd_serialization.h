//
// Simple binary serialization for placeholder ICD messages.
//
#ifndef NATURE_ICD_SERIALIZATION_H
#define NATURE_ICD_SERIALIZATION_H

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "nature/messaging/icd_messages.h"

namespace nature {
namespace transport {

namespace detail {

inline void write_bytes(std::vector<uint8_t> &out, const void *data, size_t size) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(data);
  out.insert(out.end(), bytes, bytes + size);
}

template <typename T>
inline void write_pod(std::vector<uint8_t> &out, const T &value) {
  write_bytes(out, &value, sizeof(T));
}

inline void write_string(std::vector<uint8_t> &out, const std::string &value) {
  uint32_t len = static_cast<uint32_t>(value.size());
  write_pod(out, len);
  if (len > 0) {
    write_bytes(out, value.data(), len);
  }
}

inline bool read_bytes(const std::vector<uint8_t> &in, size_t &offset, void *data, size_t size) {
  if (offset + size > in.size()) {
    return false;
  }
  std::memcpy(data, in.data() + offset, size);
  offset += size;
  return true;
}

template <typename T>
inline bool read_pod(const std::vector<uint8_t> &in, size_t &offset, T &value) {
  return read_bytes(in, offset, &value, sizeof(T));
}

inline bool read_string(const std::vector<uint8_t> &in, size_t &offset, std::string &value) {
  uint32_t len = 0;
  if (!read_pod(in, offset, len)) {
    return false;
  }
  if (offset + len > in.size()) {
    return false;
  }
  value.assign(reinterpret_cast<const char *>(in.data() + offset), len);
  offset += len;
  return true;
}

} // namespace detail

// TODO: Add versioning, endianness handling, and checksum per ICD requirements.

inline void serialize(const nature::icd::Header &header, std::vector<uint8_t> &out) {
  detail::write_pod(out, header.stamp_ns);
  detail::write_pod(out, header.seq);
  detail::write_string(out, header.frame_id);
}

inline bool deserialize(const std::vector<uint8_t> &in, size_t &offset, nature::icd::Header &header) {
  return detail::read_pod(in, offset, header.stamp_ns) &&
         detail::read_pod(in, offset, header.seq) &&
         detail::read_string(in, offset, header.frame_id);
}

inline std::vector<uint8_t> serialize(const nature::icd::Imu &imu) {
  std::vector<uint8_t> out;
  serialize(imu.header, out);
  detail::write_bytes(out, imu.accel, sizeof(imu.accel));
  detail::write_bytes(out, imu.gyro, sizeof(imu.gyro));
  return out;
}

inline bool deserialize(const std::vector<uint8_t> &in, nature::icd::Imu &imu) {
  size_t offset = 0;
  if (!deserialize(in, offset, imu.header)) {
    return false;
  }
  return detail::read_bytes(in, offset, imu.accel, sizeof(imu.accel)) &&
         detail::read_bytes(in, offset, imu.gyro, sizeof(imu.gyro));
}

inline std::vector<uint8_t> serialize(const nature::icd::Image &image) {
  std::vector<uint8_t> out;
  serialize(image.header, out);
  detail::write_pod(out, image.width);
  detail::write_pod(out, image.height);
  detail::write_string(out, image.encoding);
  detail::write_pod(out, image.step);
  uint32_t size = static_cast<uint32_t>(image.data.size());
  detail::write_pod(out, size);
  if (size > 0) {
    detail::write_bytes(out, image.data.data(), size);
  }
  return out;
}

inline bool deserialize(const std::vector<uint8_t> &in, nature::icd::Image &image) {
  size_t offset = 0;
  if (!deserialize(in, offset, image.header)) {
    return false;
  }
  uint32_t size = 0;
  if (!detail::read_pod(in, offset, image.width) ||
      !detail::read_pod(in, offset, image.height) ||
      !detail::read_string(in, offset, image.encoding) ||
      !detail::read_pod(in, offset, image.step) ||
      !detail::read_pod(in, offset, size)) {
    return false;
  }
  if (offset + size > in.size()) {
    return false;
  }
  image.data.assign(in.begin() + static_cast<long>(offset),
                    in.begin() + static_cast<long>(offset + size));
  return true;
}

inline std::vector<uint8_t> serialize(const nature::icd::Odometry &odom) {
  std::vector<uint8_t> out;
  serialize(odom.header, out);
  detail::write_bytes(out, odom.position, sizeof(odom.position));
  detail::write_bytes(out, odom.orientation, sizeof(odom.orientation));
  detail::write_bytes(out, odom.linear_vel, sizeof(odom.linear_vel));
  detail::write_bytes(out, odom.angular_vel, sizeof(odom.angular_vel));
  return out;
}

inline bool deserialize(const std::vector<uint8_t> &in, nature::icd::Odometry &odom) {
  size_t offset = 0;
  if (!deserialize(in, offset, odom.header)) {
    return false;
  }
  return detail::read_bytes(in, offset, odom.position, sizeof(odom.position)) &&
         detail::read_bytes(in, offset, odom.orientation, sizeof(odom.orientation)) &&
         detail::read_bytes(in, offset, odom.linear_vel, sizeof(odom.linear_vel)) &&
         detail::read_bytes(in, offset, odom.angular_vel, sizeof(odom.angular_vel));
}

inline std::vector<uint8_t> serialize(const nature::icd::OccupancyGrid &grid) {
  std::vector<uint8_t> out;
  serialize(grid.header, out);
  detail::write_pod(out, grid.resolution);
  detail::write_pod(out, grid.width);
  detail::write_pod(out, grid.height);
  detail::write_pod(out, grid.origin_x);
  detail::write_pod(out, grid.origin_y);
  uint32_t size = static_cast<uint32_t>(grid.data.size());
  detail::write_pod(out, size);
  if (size > 0) {
    detail::write_bytes(out, grid.data.data(), size);
  }
  return out;
}

inline bool deserialize(const std::vector<uint8_t> &in, nature::icd::OccupancyGrid &grid) {
  size_t offset = 0;
  if (!deserialize(in, offset, grid.header)) {
    return false;
  }
  uint32_t size = 0;
  if (!detail::read_pod(in, offset, grid.resolution) ||
      !detail::read_pod(in, offset, grid.width) ||
      !detail::read_pod(in, offset, grid.height) ||
      !detail::read_pod(in, offset, grid.origin_x) ||
      !detail::read_pod(in, offset, grid.origin_y) ||
      !detail::read_pod(in, offset, size)) {
    return false;
  }
  if (offset + size > in.size()) {
    return false;
  }
  grid.data.assign(in.begin() + static_cast<long>(offset),
                   in.begin() + static_cast<long>(offset + size));
  return true;
}

} // namespace transport
} // namespace nature

#endif // NATURE_ICD_SERIALIZATION_H
