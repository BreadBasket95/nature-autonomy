/**
 * \class PfPlanner
 *
 * Class for the potential field planner. 
 * 
 * author: Atsushi Sakai (@Atsushi_twi)
 * Ref:
 * https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
 *
 * Original source code from:
 * https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/PotentialFieldPlanning\
 *
 * Modified by CTG to be faster, integrate with the stack messaging, and work with an Occupancy grid.
 *
 * \author Chris Goodin
 *
 * \date 1/19/2022
 */
#ifndef PF_PLANNER_H
#define PF_PLANNER_H
// c++ includes
#include <vector>
// messaging includes
#include "nature/messaging/message_types.h"

namespace nature {
namespace planning{

class PfPlanner {
public:
	/**
	 * @brief Construct the potential field planner.
	 * @details Initializes default gains and clears internal path buffers.
	 */ 
	PfPlanner();

	/**
	 * @brief Plan a path using potential fields.
	 * @param grid Occupancy grid representing obstacles.
	 * @param odom Current vehicle odometry.
	 * @return Planned path as a message.
	 * @details Computes attractive and repulsive potentials and generates a path
	 *          in the local ENU frame for the local planner node.
	 */ 
	nature::msg::Path Plan(nature::msg::OccupancyGrid grid, nature::msg::Odometry odom);

	/**
	 * @brief Set the segmentation grid to use for terrain costs.
	 * @param seg_grid Segmentation occupancy grid.
	 * @details Enables semantic-aware planning when provided.
	 */
	void SetSegGrid(nature::msg::OccupancyGrid seg_grid){ seg_grid_ = seg_grid; seg_grid_set_ = true; }

	/**
	 * @brief Set the goal point in the local ENU frame.
	 * @param gx Goal x coordinate.
	 * @param gy Goal y coordinate.
	 * @details Used as the attractive potential source.
	 */
	void SetGoal(float gx, float gy);

	/**
	 * @brief Set the strength of the repulsive potential.
	 * @param eta Repulsive gain.
	 * @details Higher values push paths further from obstacles.
	 */
	void SetEta(float eta){eta_ = eta;}

	/**
	 * @brief Set the strength of the attractive potential.
	 * @param kp Attractive gain.
	 * @details Higher values pull paths more directly toward the goal.
	 */
	void SetKp(float kp){kp_ = kp;}

	/**
	 * @brief Set the obstacle cutoff distance.
	 * @param cutoff_dist Distance at which repulsion becomes zero.
	 * @details Obstacles farther than this range are ignored.
	 */
	void SetCutoffDistance(float cutoff_dist){ obs_cutoff_dist_ = cutoff_dist; }

	/**
	 * @brief Set the inner cutoff distance.
	 * @param inner_cutoff Distance within which obstacles are ignored.
	 * @details Prevents singularities in repulsive potential near zero.
	 */
	void SetInnerCutoff(float inner_cutoff){ inner_cutoff_dist_ = inner_cutoff; }

	/**
	 * @brief Set the obstacle cost threshold.
	 * @param oct Occupancy value considered an obstacle (0-100).
	 * @details Used to filter grid cells into obstacle lists.
	 */
	void SetObstacleCostThreshold(int oct){ obs_cost_thresh_ = oct; }
	
private:
	/**
	 * @brief Compute hypotenuse length.
	 * @param x Delta x.
	 * @param y Delta y.
	 * @return Euclidean distance.
	 * @details Helper used in potential calculations.
	 */
	float Hypot(float x, float y);
    
	/**
	 * @brief Compute attractive potential at a position.
	 * @param x X coordinate.
	 * @param y Y coordinate.
	 * @param gx Goal x coordinate.
	 * @param gy Goal y coordinate.
	 * @return Attractive potential value.
	 * @details Typically proportional to distance to goal.
	 */
	float CalcAttractivePotential(float x, float y, float gx, float gy);

	/**
	 * @brief Compute repulsive potential at a position.
	 * @param x X coordinate.
	 * @param y Y coordinate.
	 * @param ox List of obstacle x positions.
	 * @param oy List of obstacle y positions.
	 * @return Repulsive potential value.
	 * @details Uses nearest obstacle distance with cutoff parameters.
	 */
	float CalcRepulsivePotential(float x, float y, std::vector<float> ox, std::vector<float> oy);

	/**
	 * @brief Generate the motion model steps.
	 * @param step Step size in meters.
	 * @return Vector of motion steps (dx,dy).
	 * @details Used to expand candidate moves in the field.
	 */
	std::vector<std::vector<float> > GetMotionModel(float step);

	/**
	 * @brief Run the potential field planning loop.
	 * @param minx Minimum x in the grid.
	 * @param miny Minimum y in the grid.
	 * @param reso Grid resolution.
	 * @param sx Start x.
	 * @param sy Start y.
	 * @param gx Goal x.
	 * @param gy Goal y.
	 * @param ox Obstacle x positions.
	 * @param oy Obstacle y positions.
	 * @details Populates rx_/ry_ with the resulting path.
	 */
	void PotentialFieldPlanning(float minx, float miny, float reso, float sx, float sy, float gx, float gy, std::vector<float> ox, std::vector<float> oy);

	float goal_x_;
	float goal_y_;
	float kp_;
	float eta_;
	float obs_cutoff_dist_;
	float inner_cutoff_dist_;
	std::vector<float> rx_;
	std::vector<float> ry_;
	std::vector<float> old_rx_;
	std::vector<float> old_ry_;
	nature::msg::OccupancyGrid seg_grid_;
	bool seg_grid_set_;
	int obs_cost_thresh_;
};

} // namespace planning
} // namespace nature

#endif
