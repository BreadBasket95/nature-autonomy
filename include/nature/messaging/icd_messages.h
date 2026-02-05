//
// Placeholder ICD message definitions for ZMQ transport integration.
//
#ifndef NATURE_ICD_MESSAGES_H
#define NATURE_ICD_MESSAGES_H

#include <cstdint>
#include <string>
#include <vector>

namespace nature {
namespace icd {

struct Header {
  uint64_t stamp_ns = 0;
  uint32_t seq = 0;
  std::string frame_id;
};

struct Imu {
  Header header;
  double accel[3] = {0.0, 0.0, 0.0};
  double gyro[3] = {0.0, 0.0, 0.0};
  // TODO: Extend with covariances and orientation per ICD spec.
};

struct Image {
  Header header;
  uint32_t width = 0;
  uint32_t height = 0;
  std::string encoding;
  uint32_t step = 0;
  std::vector<uint8_t> data;
  // TODO: Add pixel format enums, compression flags, and endianness.
};

struct Odometry {
  Header header;
  double position[3] = {0.0, 0.0, 0.0};
  double orientation[4] = {0.0, 0.0, 0.0, 1.0};
  double linear_vel[3] = {0.0, 0.0, 0.0};
  double angular_vel[3] = {0.0, 0.0, 0.0};
  // TODO: Add covariance and frame linkage fields per ICD spec.
};

struct OccupancyGrid {
  Header header;
  float resolution = 1.0f;
  uint32_t width = 0;
  uint32_t height = 0;
  float origin_x = 0.0f;
  float origin_y = 0.0f;
  std::vector<int8_t> data;
  // TODO: Add origin orientation and map metadata fields per ICD spec.
};

} // namespace icd
} // namespace nature

#endif // NATURE_ICD_MESSAGES_H
