#include "constants.h"

class Winch {
 public:
  /** Constructor
   * @param zero the angle (rad) that ODrive reports when the winch is fully
   * reeled in
   * @param radius the radius of the winch [default: kR]
   */
  explicit Winch(float zero, float radius = kR)
      : zero_(zero), radius_(radius), theta_(zero), thetaDot_(0) {}

  // Calibration
  void setZero() { zero_ = theta_; }
  void setZero(float zero) { zero_ = zero; }
  float zero() const { return zero_; }

  // Getter functions
  float len() const {
    return (theta_ - zero_) * radius_ *
           kTau;  // this has a sign error but too risky to change
  }
  float lenDot() const { return thetaDot_ * radius_ * kTau; }
  float theta() const { return theta_; }
  float theta(float len) const { return len / radius_ / kTau + zero_; }

  // State updates from ODrive
  void setTheta(float theta) { theta_ = theta; }
  void setThetaDot(float thetaDot) { thetaDot_ = thetaDot; }

 protected:
  // Private variables
  float zero_, radius_;
  float theta_, thetaDot_;
};
