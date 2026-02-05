/**
* \class PidController
*
* A simple Proportional-Integral-Derivative (PID) controller.
* Controller is generic, but used for speed control in this application.
*
* \author Chris Goodin
*
* \date 8/31/2020
*/
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
//#include <fstream>

namespace nature {
namespace control{

class PidController{
 public:
  /**
   * @brief Construct a PID controller with default gains.
   * @details Initializes gains and state for use in speed control.
   */
  PidController();

  /**
   * @brief Destroy the PID controller.
   * @details Default cleanup of internal state.
   */
  ~PidController();

  /**
   * @brief Compute the control output for a measured value.
   * @param measured_value Current measured value.
   * @param dt Time step in seconds.
   * @return Control output value.
   * @details Computes proportional, integral, and derivative terms, with
   *          optional overshoot limiting and feed-forward support.
   */
  double GetControlVariable(double measured_value, double dt);

  /**
   * @brief Set the target setpoint.
   * @param setpoint Desired setpoint value.
   * @details Used as the reference for error computation.
   */
  void SetSetpoint(double setpoint){setpoint_ = setpoint;}

  /**
   * @brief Set the proportional gain.
   * @param kp Proportional gain.
   * @details Scales the instantaneous error term.
   */
  void SetKp(double kp){kp_=kp;}

  /**
   * @brief Set the integral gain.
   * @param ki Integral gain.
   * @details Scales the accumulated error term.
   */
  void SetKi(double ki){ki_ = ki;}

  /**
   * @brief Set the derivative gain.
   * @param kd Derivative gain.
   * @details Scales the error rate term.
   */
  void SetKd(double kd){kd_ = kd;}

  /**
   * @brief Enable or disable overshoot limiting behavior.
   * @param osl True to enable overshoot limiting.
   * @details When enabled, integral action may be clamped around setpoint.
   */
  void SetOvershootLimiter(bool osl){ overshoot_limiter_ = osl; }
  
  /**
   * @brief Force control output to remain positive.
   * @param sp True to clamp output to non-negative values.
   * @details Useful for throttle-only controllers.
   */
  void SetStayPositive(bool sp){ stay_positive_ = sp; }

  /**
   * @brief Enable or disable feed-forward contribution.
   * @param uff True to enable feed-forward.
   * @details When enabled, feed-forward model terms are added to PID output.
   */
  void SetUseFeedForward(bool uff){ use_feed_forward_ = uff; }

  /**
   * @brief Set feed-forward model parameters.
   * @param a0 Constant term.
   * @param a1 Linear term.
   * @param a2 Quadratic term.
   * @details Configures a polynomial feed-forward model for the controller.
   */
  void SetForwardModelParams(double a0, double a1, double a2){
    ff_a0_ = a0;
    ff_a1_ = a1;
    ff_a2_ = a2;
  }

 private:
  double kp_;
  double ki_;
  double kd_;
  double setpoint_;
  double previous_error_;
  double integral_;
  bool overshoot_limiter_;
  bool crossed_setpoint_;
  bool stay_positive_;

  // feed forward model parameters
  bool use_feed_forward_;
  double ff_a1_;
  double ff_a2_;
  double ff_a0_;
  //std::ofstream fout_;
};

} // namespace control
} // namespace nature
#endif
