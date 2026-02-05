/**
 * \class Path
 *
 * Class for the path planner. 
 * Adapted for use in off-road with the stack messaging from the paper:
 * 
 * Hu, X., Chen, L., Tang, B., Cao, D., & He, H. (2018). 
 * Dynamic path planning for autonomous driving on various roads with avoidance of static and moving obstacles. 
 * Mechanical Systems and Signal Processing, 100, 482-500.
 *
 * \author Chris Goodin
 *
 * \date 9/3/2020
 */
#ifndef SPLINE_PLANNER_H
#define SPLINE_PLANNER_H

#include <vector>
#include "nature/planning/local/spline_path.h"
#include "nature/planning/local/candidate.h"
// messaging includes
#include "nature/messaging/message_types.h"

namespace nature {
namespace planning{

class Planner {
public:
	/**
	 * @brief Construct the spline-based local planner.
	 * @details Initializes default weights and state used for candidate generation.
	 */ 
	Planner();

	/**
	 * @brief Set the centerline path for planning.
	 * @param path Path object describing the road centerline.
	 * @details The centerline is used as the reference for candidate generation.
	 */
	void SetCenterline(Path path) { path_ = path; }

	/**
	 * @brief Generate a set of candidate paths.
	 * @param npaths Number of paths to generate.
	 * @param s_start Arc length along the centerline at the start.
	 * @param rho_start Lateral offset at the start.
	 * @param theta_start Vehicle heading relative to east at the start.
	 * @param s_look_ahead Forward planning distance along the path.
	 * @param max_steer_angle Maximum steering angle in radians.
	 * @param vehicle_width Vehicle width in meters.
	 * @details Builds cubic polynomials that satisfy boundary conditions for
	 *          lateral offset and heading; used before scoring.
	 */
	void GeneratePaths(int npaths, float s_start, float rho_start, float theta_start, float s_look_ahead, 
	float max_steer_angle, float vehicle_width);

	/**
	 * @brief Get the list of generated candidate paths.
	 * @return Vector of Candidate objects.
	 * @details Used by visualization and downstream evaluation.
	 */ 
	std::vector<Candidate> GetCandidates() { return candidates_; }

	/**
	 * @brief Score candidate paths given occupancy and odometry.
	 * @param grid Occupancy grid for obstacle costs.
	 * @param segmentation_grid Segmentation grid for terrain costs.
	 * @param odom Current vehicle odometry.
	 * @return True if scoring succeeded.
	 * @details Computes comfortability, safety, and adherence metrics for each
	 *          candidate and selects the best path.
	 */ 
	bool CalculateCandidateCosts(nature::msg::OccupancyGrid grid, nature::msg::OccupancyGrid segmentation_grid, nature::msg::Odometry odom);

	/**
	 * @brief Dilate the occupancy grid with a square mask.
	 * @param grid Occupancy grid to dilate (modified in place).
	 * @param x Mask radius; mask size is (x+1)*(x+1).
	 * @param llx Lower-left x coordinate of the grid.
	 * @param lly Lower-left y coordinate of the grid.
	 * @param urx Upper-right x coordinate of the grid.
	 * @param ury Upper-right y coordinate of the grid.
	 * @details Expands obstacles to account for vehicle footprint.
	 */
	void DilateGrid(nature::msg::OccupancyGrid &grid, int x, float llx, float lly, float urx, float ury);

	/**
	 * @brief Sample the optimal path ahead of the current position.
	 * @param s_step Arc length ahead to sample.
	 * @return Cartesian point in ENU coordinates.
	 * @details Used by controllers to compute lookahead targets.
	 */
	utils::vec2 GetNextPoint(float s_step);

	/**
	 * @brief Get the heading angle at arc length s along the optimal path.
	 * @param s Arc length along the optimal path.
	 * @return Heading angle in radians.
	 * @details Used by controllers for steering commands.
	 */
	float GetAngleAt(float s);

	/**
	 * @brief Return the currently selected optimal path.
	 * @return Candidate representing the best path.
	 * @details Selected after scoring candidate paths.
	 */
	Candidate GetBestPath(){return last_selected_;}

	/**
	 * @brief Set the weight on the comfortability factor.
	 * @param w Desired weight.
	 * @details Higher weights penalize uncomfortable trajectories.
	 */ 
	void SetComfortabilityWeight(float w){ w_c_ = w; }

	/**
	 * @brief Set the weight on the static safety factor.
	 * @param w Desired weight.
	 * @details Higher weights penalize proximity to static obstacles.
	 */ 
	void SetStaticSafetyWeight(float w){ w_s_ = w; }

	/**
	 * @brief Set the weight on the dynamic safety factor.
	 * @param w Desired weight.
	 * @details Higher weights penalize dynamic risk and speed constraints.
	 */ 
	void SetDynamicSafetyWeight(float w){ w_d_ = w; }

	/**
	 * @brief Set the weight on the path adherence factor.
	 * @param w Desired weight.
	 * @details Higher weights favor staying close to the centerline.
	 */ 
	void SetPathAdherenceWeight(float w){ w_r_ = w; }

	/**
	 * @brief Ignore collisions before a given distance along the path.
	 * @param s_no_coll_before Arc length before which collisions are ignored.
	 * @details Used to avoid penalizing the immediate vicinity of the vehicle.
	 */
	void SetIgnoreCollBeforeDist(float s_no_coll_before) { s_no_coll_before_ = s_no_coll_before; }

	/**
	 * @brief Enable or disable blending between neighboring candidates.
	 * @param use_blend True to blend costs across adjacent paths.
	 * @details Blending smooths the cost landscape across candidates.
	 */
  	void SetUseBlend(bool use_blend){ use_blend_ = use_blend; }

	/**
	 * @brief Get the comfortability weight.
	 * @return Weight value.
	 * @details Used for diagnostics and tuning.
	 */
	float GetComfortabilityWeight() const { return w_c_; }
	/**
	 * @brief Get the static safety weight.
	 * @return Weight value.
	 * @details Used for diagnostics and tuning.
	 */
	float GetStaticSafetyWeight() const { return w_s_; }
	/**
	 * @brief Get the dynamic safety weight.
	 * @return Weight value.
	 * @details Used for diagnostics and tuning.
	 */
	float GetDynamicSafetyWeight() const { return w_d_; }
	/**
	 * @brief Get the path adherence weight.
	 * @return Weight value.
	 * @details Used for diagnostics and tuning.
	 */
	float GetPathAdherenceWeight() const { return w_r_; }
	/**
	 * @brief Get the segmentation cost weight.
	 * @return Weight value.
	 * @details Used for diagnostics and tuning.
	 */
	float GetSegmentationWeight() const { return w_t_; }

	/**
	 * @brief Set the consistency factor weight for comfortability.
	 * @param w Desired weight.
	 * @details Influences smoothing of curvature changes.
	 */ 
	void SetConsistencyFactorWeight(float w){ b_= w; }

    /**
    * @brief Set the terrain segmentation cost weight.
    * @param w Desired weight.
    * @details Penalizes candidates that traverse undesirable terrain classes.
    */
    void SetSegmentationFactorWeight(float w){ w_t_ = w; }

	/**
	 * @brief Set the curvature factor weight for comfortability.
	 * @param w Desired weight.
	 * @details Penalizes high curvature trajectories.
	 */ 
	void SetCurvatureFactorWeight(float w){ a_ = w; }

	/**
	 * @brief Set the averaging window size for static safety.
	 * @param np Number of adjacent paths to average.
	 * @details Defaults based on vehicle width when not explicitly set.
	 */ 
	void SetAveragingWindowSize(int np){ averaging_window_size_ = np; }

	/**
	 * @brief Set the integration step size for curvature calculations.
	 * @param ds Integration step size in meters.
	 * @details Smaller steps improve accuracy at a performance cost.
	 */
	void SetArcLengthIntegrationStep(float ds){ ds_ = ds; }

	/**
	 * @brief Set the dynamic safety parameters.
	 * @param alpha Limit of lateral acceleration.
	 * @param k Safety gain for speed adjustment.
	 * @param v Reference speed for the path.
	 * @details Parameters are used when computing dynamic safety costs.
	 */ 
	void SetDynamicSafetyParams(float alpha, float k, float v){
		alpha_max_ = alpha;
		k_safe_ = k;
		v_curve_ = v;
	}

private:
	// private methods
	/**
	 * @brief Compute cubic polynomial coefficients for a candidate path.
	 * @param rho_start Lateral offset at the start.
	 * @param theta_start Heading angle at the start.
	 * @param s_end Arc length at the end.
	 * @param rho_end Lateral offset at the end.
	 * @return Coefficient vector for the cubic polynomial.
	 * @details Used by GeneratePaths to build candidate curves.
	 */
	std::vector<float> CalcCoeffs(float rho_start, float theta_start, float s_end, float rho_end);
	/**
	 * @brief Compute comfortability scores for all candidates.
	 * @details Uses curvature and consistency metrics to score smoothness.
	 */
	void CalculateComfortability();
	/**
	 * @brief Compute static safety and segmentation costs.
	 * @param grid Occupancy grid for obstacles.
	 * @param segmentation_grid Segmentation grid for terrain classes.
	 * @details Evaluates collision risk and semantic penalties.
	 */
	void CalculateStaticSafetyAndSegCost(const nature::msg::OccupancyGrid & grid,const nature::msg::OccupancyGrid & segmentation_grid);
	/**
	 * @brief Compute path adherence (rho) costs.
	 * @details Penalizes large deviations from the centerline.
	 */
	void CalculateRhoCost();
	/**
	 * @brief Compute dynamic safety costs.
	 * @param odom Vehicle odometry for speed and heading.
	 * @details Penalizes trajectories that violate dynamic constraints.
	 */
	void CalculateDynamicSafety(nature::msg::Odometry odom);
	/**
	 * @brief Get the total cost for a candidate path.
	 * @param pathnum Index of the candidate.
	 * @return Total weighted cost.
	 * @details Combines comfortability, safety, and adherence terms.
	 */
	float GetTotalCostOfCandidate(int pathnum);
	/**
	 * @brief Compute curve info for a candidate at a given arc length.
	 * @param candidate Candidate path to evaluate.
	 * @param s Arc length along the candidate.
	 * @param base_ca Base curvature/angle info from centerline.
	 * @return CurveInfo for the candidate at s.
	 * @details Used when evaluating comfortability and curvature.
	 */
	CurveInfo InfoOfCurve(Candidate candidate, float s, CurveInfo base_ca);

	// centerline
	Path path_;

	// candidates
	std::vector<Candidate> candidates_;

	// optimal path
	Candidate last_selected_;

	// state variables to track
	bool first_iter_;
	float s_max_;
	float rho_max_;
	float s_start_;

	// Planner parameters
	float w_c_;
	float w_s_;
	float w_d_;
	float w_r_;
	float w_t_;
	float alpha_max_; 
	float k_safe_;
	float v_curve_;
	float a_;
	float b_;
	float ds_;
	float s_no_coll_before_;
	int averaging_window_size_;
	bool use_blend_;
};

} // namespace planning
} // namespace nature

#endif
