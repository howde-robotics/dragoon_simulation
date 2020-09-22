#pragma once

/**
 * @brief a simple PID controller
 *
 * Takes in a setPoint and does this
 *
 */
class myPidController
{
public:
  /// set the respective p, i, and d gains
  myPidController(double p, double i, double d);
  ~myPidController()
  {
  }

  /**
   * @brief calculate the PID output signal
   *
   * @param setPoint desired set point that you want the system to go to, unitless
   * @return double
   */
  double calcPid(double setPoint);

private:
  /// proportional gain, unitless
  double p_;
  /// integral gain, unitless
  double i_;
  /// differential gain, unitless
  double d_;
  /// the summation of the (i * error), unitless
  double i_sum_;
};

myPidController::myPidController(double p, double i, double d) : p_(p), i_(i), d_(d), i_sum_(0.0)
{
  // Do stuff in Constructor
}

double myPidController::calcPid(double setPoint)
{
  double output;
  // Do stuff in function
  return output;
}