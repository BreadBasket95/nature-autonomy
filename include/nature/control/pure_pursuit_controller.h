/**
* \class PurePursuitController
*
* A pure-pursuit vehicle control that follows an input vehicle
* trajectory in 2D space. 
* 
* See "Implementation of the Pure Pursuit Path Tracking Algorithm"
* by Craig Coulter, CMU-RI-TR-92-01
* 
* and
* 
* "Automatic Steering Methods for Autonomous Automobile Path Tracking"
* by Jarrod M. Snider, CMU-RI-TR-09-08
*
* \author Chris Goodin
*
* \date 8/31/2020
*/
#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include "nature/control/pid_controller.h"
#include "nature/messaging/message_types.h"
#include "nature/nature_utils.h"

namespace nature {
namespace control{

class PurePursuitController {
public:
	/**
	 * @brief Construct the pure pursuit controller.
	 * @details Initializes default parameters and PID controller settings.
	 */
	PurePursuitController();

	/**
	* @brief Compute a driving command from a trajectory.
	* @param traj Desired trajectory; first point is the current vehicle state.
	* @param goal Output goal point selected along the trajectory.
	* @return Twist command with throttle/brake and steering.
	* @details Implements pure pursuit to compute steering and uses PID for speed.
	*/
	nature::msg::Twist GetDcFromTraj(nature::msg::Path traj, utils::vec2 & goal);

	/**
	* @brief Set the vehicle wheelbase.
	* @param wb Wheelbase in meters.
	* @details Affects steering geometry for Ackermann control.
	*/
	void SetWheelbase(float wb) { wheelbase_ = wb; }

	/**
	* @brief Set the maximum steering angle.
	* @param st Maximum steering angle in radians.
	* @details Used to clamp steering commands.
	*/
	void SetMaxSteering(float st) { max_steering_angle_ = st; }

	/** 
	* @brief Set the minimum look-ahead distance.
	* @param min_la Minimum look-ahead distance in meters.
	* @details Bounds the lookahead used for pure pursuit target selection.
	*/
	void SetMinLookAhead(float min_la) { min_lookahead_ = min_la; }

	/**
	* @brief Set the maximum look-ahead distance.
	* @param max_la Maximum look-ahead distance in meters.
	* @details Bounds the lookahead used for pure pursuit target selection.
	*/
	void SetMaxLookAhead(float max_la) { max_lookahead_ = max_la; }

	/** 
	* @brief Set the steering gain factor.
	* @param k Gain factor.
	* @details Scales steering response for pure pursuit.
	*/
	void SetSteeringParam(float k) { k_ = k; }

	/**
	* @brief Set the maximum stable speed.
	* @param speed Maximum speed in m/s.
	* @details Controller limits throttle to stay below this speed.
	*/
	void SetMaxStableSpeed(float speed) { max_stable_speed_ = speed; }

	/**
	* @brief Set the desired speed.
	* @param speed Desired speed in m/s.
	* @details Updates the PID controller setpoint.
	*/
	void SetDesiredSpeed(float speed) {
		desired_speed_ = speed;
		speed_controller_.SetSetpoint(speed);
	}

	/**
	* @brief Set PID gains for speed control.
	* @param kp Proportional coefficient.
	* @param ki Integral coefficient.
	* @param kd Derivative coefficient.
	* @details Tuning these gains affects throttle response.
	*/
	void SetSpeedControllerParams(float kp, float ki, float kd) {
		speed_controller_.SetKp(kp);
		speed_controller_.SetKi(ki);
		speed_controller_.SetKd(kd);
	}

	/**
	* @brief Set the current vehicle position.
	* @param x Current x-coordinate in ENU.
	* @param y Current y-coordinate in ENU.
	* @details Used to compute lookahead target and heading error.
	*/
	void SetVehiclePosition(float x, float y) {
		veh_x_ = x;
		veh_y_ = y;
	}

	/**
	* @brief Set the current vehicle speed.
	* @param speed Current speed in m/s.
	* @details Updates internal state for PID and dynamic safety.
	*/
	void SetVehicleSpeed(float speed);

	/**
	* @brief Set the current vehicle heading.
	* @param heading Heading in radians.
	* @details Used to compute steering error.
	*/
	void SetVehicleOrientation(float heading) {
		veh_heading_ = heading;
	}

	/**
	 * @brief Set vehicle position, orientation, and speed from odometry.
	 * @param state Vehicle odometry message.
	 * @details Convenience wrapper for updating multiple state fields.
	 */
	void SetVehicleState(nature::msg::Odometry state);

	/**
	* @brief Set a scale factor for output throttle.
	* @param tc Throttle scale factor.
	* @details Defaults to 1.0; use sparingly as PID gains should handle speed.
	*/
	void SetThrottleCoeff(float tc){ throttle_coeff_ = tc; }

	/**
	* @brief Enable or disable skid-steered control.
	* @param skid_steered True for skid steering, false for Ackermann.
	* @details Switches between control models.
	*/
	void IsSkidSteered(bool skid_steered){ skid_steered_ = skid_steered; }

	/**
	* @brief Set parameters for the skid-steering control model.
	* @param kl Lateral gain (Kx and Ky).
	* @param kt Heading gain (Ktheta).
	* @details Used by the skid steering controller formulation.
	*/
	void SetSkidSteerParams(float kl, float kt){
		kx_ = kl;
		ky_ = kl;
		k_theta_ = kt;
	}

	/**
	 * @brief Access the PID speed controller.
	 * @return Pointer to the internal PID controller.
	 * @details Allows external tuning and inspection.
	 */
	PidController *GetPidSpeedController(){ return &speed_controller_; }

private:
	bool skid_steered_;
	/**
	 * @brief Compute Ackermann steering command.
	 * @param alpha Heading error angle.
	 * @param lookahead Lookahead distance.
	 * @param curr_dir Current direction vector.
	 * @param target_speed Target speed in m/s.
	 * @return Twist command for Ackermann steering.
	 * @details Implements pure pursuit Ackermann geometry.
	 */
	nature::msg::Twist GetDcAckermann(float alpha, float lookahead, utils::vec2 curr_dir, float target_speed);
	/**
	 * @brief Compute skid-steer command.
	 * @param dx X error to goal.
	 * @param dy Y error to goal.
	 * @param dtheta Heading error.
	 * @return Twist command for skid steering.
	 * @details Uses a non-holonomic control law for skid-steered vehicles.
	 */
	nature::msg::Twist GetDcSkid(float dx, float dy, float dtheta);

	// steering parameters for the skid steered model
	float kx_;
	float ky_;
	float k_theta_;

	float wheelbase_; //meters
	float max_steering_angle_; //radians
	float min_lookahead_; //meters
	float max_lookahead_; //meters
	float k_; //unitless
	float desired_speed_; // m/s
	float max_stable_speed_;
	float throttle_coeff_;
	PidController speed_controller_;

	//current vehicle state info
	float veh_x_;
	float veh_y_;
	float veh_heading_;
	float veh_speed_;
	float vx_;
	float vy_;
	float current_angular_velocity_;
};

} // namespace control
} // namespace nature

#endif
