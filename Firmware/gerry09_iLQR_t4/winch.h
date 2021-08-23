/**
 * The Winch class keeps track of the state of a single winch and handles e.g.
 * conversions between rotations and length.
 */

#pragma once

#include "arduino_test_utils.h"
#include "constants.h"
#include "utils.h"

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
  // Units of revolutions and rev/s!!!
  float theta() const { return theta_; }
  float thetaDot() const { return thetaDot_; }
  float theta(float len) const { return len / radius_ / kTau + zero_; }
  float thetaDot(float lenDot) const { return lenDot / radius_ / kTau; }

  // Check if valid
  bool isThetaValid() const { return theta_.isValid(); }
  bool isThetaDotValid() const { return thetaDot_.isValid(); }
  bool isThetaValid(uint64_t th) const { return theta_.isValid(th); }
  bool isThetaDotValid(uint64_t th) const { return thetaDot_.isValid(th); }

  // State updates from ODrive
  void setTheta(float theta) { theta_ = theta; }
  void setThetaDot(float thetaDot) { thetaDot_ = thetaDot; }

 protected:
  // Private variables
  float zero_, radius_;
  Timestamped<float> theta_, thetaDot_;
};
