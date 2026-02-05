//
// Placeholder message definitions to stand in for ICD structs while we refactor the messaging layer.
//
// These are intentionally minimal but mirror the field layout that the rest of the stack expects.
//
#ifndef NATURE_PLACEHOLDER_MESSAGES_H
#define NATURE_PLACEHOLDER_MESSAGES_H

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace nature {
namespace msg {

struct Header {
  int32_t seq = 0;
  double stamp = 0.0;
  std::string frame_id;
  double toSec() const { return stamp; }
};

struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  Vector3() = default;
  Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Vector3 operator+(const Vector3 &other) const { return Vector3(x + other.x, y + other.y, z + other.z); }
  Vector3 operator-(const Vector3 &other) const { return Vector3(x - other.x, y - other.y, z - other.z); }
};

struct PointStamped {
  Header header;
  Point point;
};

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose {
  Point position;
  Quaternion orientation;
};

struct PoseWithCovariance {
  Pose pose;
  std::array<double, 36> covariance = {};
};

struct Twist {
  Vector3 linear;
  Vector3 angular;
};

struct TwistWithCovariance {
  Twist twist;
  std::array<double, 36> covariance = {};
};

struct Point32 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct PointCloud {
  Header header;
  std::vector<Point32> points;
  struct Channel {
    std::string name;
    std::vector<float> values;
  };
  std::vector<Channel> channels;
};

struct PointField {
  std::string name;
  uint32_t offset = 0;
  uint8_t datatype = 0;
  uint32_t count = 1;
  static constexpr uint8_t FLOAT32 = 7;
  static constexpr uint8_t FLOAT64 = 8;
};

struct PointCloud2 {
  Header header;
  uint32_t height = 1;
  uint32_t width = 0;
  std::vector<PointField> fields;
  bool is_bigendian = false;
  uint32_t point_step = 0;
  uint32_t row_step = 0;
  std::vector<uint8_t> data;
  bool is_dense = true;
};

struct PoseStamped {
  Header header;
  Pose pose;
};

struct Path {
  Header header;
  std::vector<PoseStamped> poses;
};

struct Odometry {
  Header header;
  PoseWithCovariance pose;
  TwistWithCovariance twist;
};

struct OccupancyGrid {
  struct Info {
    Pose origin;
    float resolution = 1.0f;
    uint32_t width = 0;
    uint32_t height = 0;
  } info;
  Header header;
  std::vector<int8_t> data;
};

struct Marker {
  Header header;
  int32_t id = 0;
  int32_t type = 0;
  int32_t action = 0;
  struct Color { float r = 0.0f; float g = 0.0f; float b = 0.0f; float a = 1.0f; } color;
  Vector3 scale;
  Pose pose;
  std::vector<Point> points;
  std::string text;
  enum {
    LINE_LIST = 5,
    TEXT_VIEW_FACING = 9
  };
  enum {
    MODIFY = 0,
    DELETE = 2
  };
};

struct MarkerArray {
  std::vector<Marker> markers;
};

struct Float64 { double data = 0.0; };

struct Int32 { int32_t data = 0; };

struct MultiArrayDimension {
  std::string label;
  uint32_t size = 0;
  uint32_t stride = 0;
};

struct Float64MultiArray {
  struct Layout {
    std::vector<MultiArrayDimension> dim;
  } layout;
  std::vector<double> data;
};

struct NavSatFix {
  Header header;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
};

struct Clock {
  Header header;
  double clock = 0.0;
};

struct JointState {
  Header header;
  std::vector<std::string> name;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
};

} // namespace msg

namespace msg_tf {

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  Quaternion() = default;
  Quaternion(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}
};

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  Vector3() = default;
  Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Vector3 operator+(const Vector3 &other) const { return Vector3(x + other.x, y + other.y, z + other.z); }
  Vector3 operator*(double scale) const { return Vector3(x * scale, y * scale, z * scale); }
};

struct Matrix3x3 {
  double m[3][3] = {{1.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0},
                    {0.0, 0.0, 1.0}};
  Matrix3x3() = default;
  explicit Matrix3x3(const Quaternion &q) { fromQuaternion(q); }
  void fromQuaternion(const Quaternion &q) {
    double qx = q.x;
    double qy = q.y;
    double qz = q.z;
    double qw = q.w;
    double xx = qx * qx;
    double yy = qy * qy;
    double zz = qz * qz;
    double xy = qx * qy;
    double xz = qx * qz;
    double yz = qy * qz;
    double wx = qw * qx;
    double wy = qw * qy;
    double wz = qw * qz;
    m[0][0] = 1.0 - 2.0 * (yy + zz);
    m[0][1] = 2.0 * (xy - wz);
    m[0][2] = 2.0 * (xz + wy);
    m[1][0] = 2.0 * (xy + wz);
    m[1][1] = 1.0 - 2.0 * (xx + zz);
    m[1][2] = 2.0 * (yz - wx);
    m[2][0] = 2.0 * (xz - wy);
    m[2][1] = 2.0 * (yz + wx);
    m[2][2] = 1.0 - 2.0 * (xx + yy);
  }
  Vector3 operator*(const Vector3 &v) const {
    return Vector3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                   m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                   m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
  }
  void getRPY(double &roll, double &pitch, double &yaw) const {
    constexpr double kHalfPi = 1.5707963267948966;
    const double sinp = -m[2][0];
    if (sinp >= 1.0) {
      pitch = kHalfPi;
    } else if (sinp <= -1.0) {
      pitch = -kHalfPi;
    } else {
      pitch = std::asin(sinp);
    }
    roll = std::atan2(m[2][1], m[2][2]);
    yaw = std::atan2(m[1][0], m[0][0]);
  }
};

} // namespace msg_tf
} // namespace nature

#endif // NATURE_PLACEHOLDER_MESSAGES_H
