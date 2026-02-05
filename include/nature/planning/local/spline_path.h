/**
 * \class Path
 *
 * Path class for the planner. This is the 
 * equivalent of the centerline in the original planner.
 *
 * \author Chris Goodin
 *
 * \date 8/31/2020
 */
#ifndef SPLINE_PATH_H
#define SPLINE_PATH_H

#include <vector>
#include "nature/nature_utils.h"

namespace nature {
namespace planning{
	
/// Info regarding a path segment.
struct SegmentInfo {
	utils::vec2 point;
	int id;
};

/// Distance from a point to a segment, and the closest point on the segment.
struct PointSegDist {
	utils::vec2 point;
	float dist;
};

/// Curvature and tangent angle of the path.
struct CurveInfo {
	float curvature;
	float theta;
};

class Path {
public:
	/**
	 * @brief Construct an empty path.
	 * @details Leaves the waypoint list empty until Init is called.
	 */ 
	Path();

	/**
	 * @brief Construct a path from waypoints.
	 * @param points List of waypoints in 2D ENU coordinates.
	 * @details Initializes internal arc length and curvature caches.
	 */ 
	Path(std::vector<utils::vec2> points);

	/**
	 * @brief Construct a path from waypoints with lookahead culling.
	 * @param points List of waypoints in 2D ENU coordinates.
	 * @param position Current position in 2D ENU coordinates.
	 * @param la Maximum distance ahead to keep.
	 * @details Drops far-away waypoints to speed up local planner computations.
	 */ 
	Path(std::vector<utils::vec2> points, utils::vec2 position, float la);

	/**
	 * @brief Initialize a path from waypoints.
	 * @param points List of waypoints in 2D ENU coordinates.
	 * @details Computes arc lengths, angles, and curvature caches.
	 */ 
	void Init(std::vector<utils::vec2> points);

	/**
	 * @brief Initialize a path from waypoints with lookahead culling.
	 * @param points List of waypoints in 2D ENU coordinates.
	 * @param position Current position in 2D ENU coordinates.
	 * @param la Maximum distance ahead to keep.
	 * @details Drops far-away waypoints before computing path caches.
	 */ 
	void Init(std::vector<utils::vec2> points, utils::vec2 position, float la);

	/**
	 * @brief Get the total arc length of the path.
	 * @return Total length from first to last waypoint.
	 * @details Computed from the discrete arc length cache.
	 */ 
	float GetTotalLength();

	/**
	 * @brief Convert a point in (s, rho) to Cartesian coordinates.
	 * @param s Arc length parameter along the path.
	 * @param rho Lateral offset from the path.
	 * @return Cartesian point in ENU coordinates.
	 * @details Uses cached tangent and curvature to compute lateral offset.
	 */
	utils::vec2 ToCartesian(float s, float rho);

	/**
	 * @brief Convert a Cartesian point to (s, rho).
	 * @param x X coordinate in local ENU.
	 * @param y Y coordinate in local ENU.
	 * @return (s, rho) coordinates relative to the path.
	 * @details Finds the closest path segment and computes projection and offset.
	 */ 
	utils::vec2 ToSRho(float x, float y);

	/**
	 * @brief Get curvature and tangent angle at arc length s.
	 * @param s Arc length along the path.
	 * @return CurveInfo containing curvature and heading angle.
	 * @details Uses cached curvature and theta values.
	 */ 
	CurveInfo GetCurvatureAndAngle(float s);

	/**
	 * @brief Get the last waypoint on the path.
	 * @return Last waypoint in the list.
	 * @details Used by planners to detect path end conditions.
	 */
	utils::vec2 GetLastPoint() { return points_[points_.size() - 1]; }

	/**
	 * @brief Get the waypoint at a given index.
	 * @param index Index of the waypoint.
	 * @return Waypoint at index or (0,0) if out of range.
	 * @details Provides bounds checking to avoid invalid access.
	 */
	utils::vec2 GetPoint(int index) {
		utils::vec2 p(0.0f, 0.0f);
		if (index >= 0 && index < points_.size()) {
			p = points_[index];
		}
		return p;
	}

	/**
	 * @brief Return the list of waypoints on the path.
	 * @return Vector of waypoints.
	 * @details Used for visualization and debugging.
	 */ 
	std::vector<utils::vec2> GetPoints(){
		return points_;
	}

	/**
	 * @brief Get the tangent angle at arc length s.
	 * @param s Arc length along the path.
	 * @return Heading angle in radians.
	 * @details Uses cached theta values computed from the waypoint list.
	 */
	float GetTheta(float s);

	/**
	* @brief Extend the path behind the vehicle start position.
	* @param x Vehicle start X in local ENU.
	* @param y Vehicle start Y in local ENU.
	* @details Adds points behind the start to improve spline generation.
	*/
	void FixBeginning(float x, float y);

	/**
	* @brief Extend the path beyond the final waypoint.
	* @details Adds points beyond the end to smooth spline termination.
	*/
	void FixEnd();

private:
	std::vector<utils::vec2> points_;
	std::vector<float> curvature_;
	std::vector<float> theta_;
	std::vector<float> arc_length_;
	std::vector<float> discrete_lengths_;
	float max_lookahead_;
	/**
	 * @brief Compute angles, curvature, and arc length caches.
	 * @details Called after initialization and waypoint updates.
	 */
	void CalcAnglesAndCurvature();

	/**
	 * @brief Compute Menger curvature from three points.
	 * @param p0 First point.
	 * @param p1 Second point.
	 * @param p2 Third point.
	 * @return Curvature estimate.
	 * @details Uses triangle geometry; helps approximate local curvature.
	 */
	float MengerCurvature(utils::vec2 p0, utils::vec2 p1, utils::vec2 p2);
	/**
	 * @brief Compute area of a triangle defined by three points.
	 * @param a First point.
	 * @param b Second point.
	 * @param c Third point.
	 * @return Signed triangle area.
	 * @details Used by curvature calculation.
	 */
	float TriangleArea(utils::vec2 a, utils::vec2 b, utils::vec2 c);

	/**
	 * @brief Compute distance from a point to a segment.
	 * @param P First endpoint of the segment.
	 * @param Q Second endpoint of the segment.
	 * @param X Query point.
	 * @return PointSegDist containing closest point and distance.
	 * @details Used by ToSRho to find nearest segment.
	 */
	PointSegDist PointToSegmentDistance(utils::vec2 P, utils::vec2 Q, utils::vec2 X);
	/**
	 * @brief Find the segment containing the arc length s.
	 * @param s Arc length along the path.
	 * @return SegmentInfo with index and point.
	 * @details Used for interpolation between waypoints.
	 */
	SegmentInfo FindSegment(float s);

};

} // namespace planning
} // namespace nature


#endif
