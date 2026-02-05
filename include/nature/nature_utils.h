/**
 * \file nature_utils.h
 *
 * Structs and inline functions used by all the algorithms.
 *
 * \date 9/3/2020
 */
#ifndef NATURE_UTILS_H
#define NATURE_UTILS_H

#include "nature/messaging/message_types.h"
#include <iomanip>
#include <sstream>

namespace nature {
namespace utils {

struct vec2{
	/**
	 * @brief Default-construct a 2D float vector with zero components.
	 * @details Initializes x and y to 0.0f so the vector is safe for accumulation.
	 *          This type is used throughout planning and control for lightweight math.
	 */
	vec2(){
		x = 0.0f;
		y = 0.0f;
	}
	/**
	 * @brief Construct a 2D float vector from explicit components.
	 * @param x_ X component to store.
	 * @param y_ Y component to store.
	 * @details Stores the provided values directly; no normalization is performed.
	 */
	vec2(float x_, float y_){
		x = x_; 
		y = y_;
	}
	/**
	 * @brief Add two vectors component-wise.
	 * @param b Vector to add.
	 * @return Sum of this vector and b.
	 * @details Performs simple component-wise addition; used heavily in trajectory math.
	 */
	vec2 operator+(const vec2& b) { return vec2(this->x + b.x, this->y + b.y); }
	/**
	 * @brief Subtract two vectors component-wise.
	 * @param b Vector to subtract.
	 * @return Difference of this vector and b.
	 * @details Performs simple component-wise subtraction.
	 */
	vec2 operator-(const vec2& b) { return vec2(this->x - b.x, this->y - b.y); }
	/**
	 * @brief Scale the vector by a scalar.
	 * @param s Scalar multiplier.
	 * @return Scaled vector.
	 * @details Multiplies each component by s; used for gain and unit conversions.
	 */
	vec2 operator*(const float s) { return vec2(s*this->x, s*this->y); }
	/**
	 * @brief Divide the vector by a scalar.
	 * @param s Scalar divisor.
	 * @return Scaled vector.
	 * @details Divides each component by s; caller must avoid s==0.
	 */
	vec2 operator/(const float s) { return vec2(this->x/s, this->y/s); }
	float x;
	float y;
};

struct vec3{
	/**
	 * @brief Default-construct a 3D float vector with zero components.
	 * @details Initializes x, y, z to 0.0f; used throughout perception and planning.
	 */
	vec3(){
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}
	/**
	 * @brief Construct a 3D float vector from explicit components.
	 * @param x_ X component to store.
	 * @param y_ Y component to store.
	 * @param z_ Z component to store.
	 * @details Stores the provided values without normalization.
	 */
	vec3(float x_, float y_, float z_){
		x = x_; 
		y = y_;
		z = z_;
	}
	/**
	 * @brief Add two vectors component-wise.
	 * @param b Vector to add.
	 * @return Sum of this vector and b.
	 * @details Used for accumulating point clouds and trajectories.
	 */
	vec3 operator+(const vec3& b) { return vec3(this->x + b.x, this->y + b.y, this->z + b.z); }
	/**
	 * @brief Subtract two vectors component-wise.
	 * @param b Vector to subtract.
	 * @return Difference of this vector and b.
	 * @details Used to form displacement vectors.
	 */
	vec3 operator-(const vec3& b) { return vec3(this->x - b.x, this->y - b.y, this->z - b.z); }
	/**
	 * @brief Scale the vector by a scalar.
	 * @param s Scalar multiplier.
	 * @return Scaled vector.
	 * @details Multiplies each component by s.
	 */
	vec3 operator*(const float s) { return vec3(s*this->x, s*this->y, s*this->z); }
	/**
	 * @brief Divide the vector by a scalar.
	 * @param s Scalar divisor.
	 * @return Scaled vector.
	 * @details Divides each component by s; caller must avoid s==0.
	 */
	vec3 operator/(const float s) { return vec3(this->x/s, this->y/s, this->z/s); }
	/**
	 * @brief Index into the vector by component.
	 * @param idx Index 0=x, 1=y, 2=z.
	 * @return Component value for the requested index.
	 * @details Implements simple branch-based indexing used by generic math code.
	 */
  float operator[](const int idx) const {
    return idx == 0 ? x : (idx == 1 ? y : z);
  };
	float x;
	float y;
	float z;
};

struct ivec2{
	/**
	 * @brief Default-construct a 2D integer vector with zero components.
	 * @details Initializes x and y to 0 for safe use in grid indexing.
	 */
	ivec2(){
		x = 0;
		y = 0;
	}
	/**
	 * @brief Construct a 2D integer vector from explicit components.
	 * @param x_ X component to store.
	 * @param y_ Y component to store.
	 * @details Stores the provided integer values directly.
	 */
	ivec2(int x_, int y_){
		x = x_; 
		y = y_;
	}
	/**
	 * @brief Add two integer vectors component-wise.
	 * @param b Vector to add.
	 * @return Sum of this vector and b.
	 * @details Used for grid coordinate arithmetic.
	 */
	ivec2 operator+(const ivec2& b) { return ivec2(this->x + b.x, this->y + b.y); }
	/**
	 * @brief Subtract two integer vectors component-wise.
	 * @param b Vector to subtract.
	 * @return Difference of this vector and b.
	 * @details Used for grid coordinate deltas.
	 */
	ivec2 operator-(const ivec2& b) { return ivec2(this->x - b.x, this->y - b.y); }
	/**
	 * @brief Scale the integer vector by a scalar.
	 * @param s Scalar multiplier.
	 * @return Scaled vector.
	 * @details Multiplies each component by s.
	 */
	ivec2 operator*(const int s) { return ivec2(s*this->x, s*this->y); }
	/**
	 * @brief Divide the integer vector by a scalar.
	 * @param s Scalar divisor.
	 * @return Scaled vector.
	 * @details Divides each component by s; caller must avoid s==0.
	 */
	ivec2 operator/(const int s) { return ivec2(this->x/s, this->y/s); }
	int x;
	int y;
};

/**
 * @brief Compute the Euclidean length of a 2D vector.
 * @param p Vector to measure.
 * @return Magnitude of the vector.
 * @details Uses sqrt(x^2+y^2); core helper used in planners and controllers.
 */
inline float length(vec2 p){
	float r = (float)sqrt(p.x*p.x + p.y*p.y);
	return r;
}

/**
 * @brief Compute the dot product of two 2D vectors.
 * @param p First vector.
 * @param q Second vector.
 * @return Scalar dot product.
 * @details Used for projections and segment distance calculations.
 */
inline float dot(vec2 p, vec2 q){
	return (p.x*q.x+p.y*q.y);
}

/**
 * @brief Compute signed distance from a point to an infinite line.
 * @param x1 First point on the line.
 * @param x2 Second point on the line.
 * @param x0 Query point.
 * @return Signed perpendicular distance from x0 to the line through x1-x2.
 * @details Uses 2D cross-product magnitude over line length; sign encodes side.
 *          Used by local planning geometry utilities.
 */
inline float PointLineDistance(vec2 x1, vec2 x2, vec2 x0) {
	float sx1 = x0.x - x1.x;
	float sy1 = x0.y - x1.y;
	float sx2 = x0.x - x2.x;
	float sy2 = x0.y - x2.y;
	float z = sx1*sy2 - sx2*sy1;
	vec2 x21 = x2 - x1;
	float  d = z / length(x21);
	return d;
}

/**
 * Return distance from a point to a segment
 * \param ep1 First endpoint of the segment
 * \param ep2 Second endpoint of the segment
 * \param p The test point 
 */
/**
 * @brief Compute distance from a point to a line segment.
 * @param ep1 First endpoint of the segment.
 * @param ep2 Second endpoint of the segment.
 * @param p Query point.
 * @return Shortest distance from p to segment ep1-ep2.
 * @details Uses projection checks to determine whether the closest point lies
 *          on an endpoint or the interior of the segment.
 */
inline float PointToSegmentDistance(vec2 ep1, vec2 ep2, vec2 p) {
	vec2 v21 = ep2 - ep1;
	vec2 pv1 = p - ep1;
	if (dot(v21, pv1) <= 0.0) {
		float d = length(pv1);
		return d;
	}
	vec2 v12 = ep1 - ep2;
	vec2 pv2 = p - ep2;
	if (dot(v12, pv2) <= 0.0) {
		float d = length(pv2);
		return d;
	}
	float d0 = PointLineDistance(ep1, ep2, p);
	return d0;
}

/**
 * @brief Extract planar heading (yaw) from a quaternion.
 * @param orientation Quaternion expressed in the vehicle frame.
 * @return Yaw angle in radians.
 * @details Converts quaternion to roll/pitch/yaw using the placeholder tf helpers
 *          and returns yaw. Used across planners that operate in the ground plane.
 */
inline float GetHeadingFromOrientation(nature::msg::Quaternion orientation){
    nature::msg_tf::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
    nature::msg_tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
    return (float)yaw;
}

/// Convert any type to a string with zero padding
/**
 * @brief Convert an integer to a zero-padded string.
 * @param x Integer to convert.
 * @param zero_padding Minimum field width (pads with leading zeros).
 * @return String representation of the integer.
 * @details Uses iostream formatting to support logging and file naming utilities.
 */
inline std::string ToString(int x, int zero_padding){
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(zero_padding) << x;
  std::string str = ss.str();
  return str;
};

} //namespace utils
} //namespace nature

#endif
