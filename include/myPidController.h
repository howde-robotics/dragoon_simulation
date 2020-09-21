#pragma once

/** Explain this class plss
 */
class myPidController
{
public:
  myPidController(double p, double i, double d);
  ~myPidController()
  {
  }

  double calcPid(double setPoint);

private:
  double p_, i_, d_;
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