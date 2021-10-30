/**
 * The Winch class keeps track of the state of a single winch and handles e.g.
 * conversions between rotations and length.
 *
 * ATTENTION: By convention, in the code "longer" causes the length to be more
 * negative.  e.g. the total range of the winch should be approx 0 to -3.8.
 * This is because of two reasons:
 *  1. Force and acceleration have the same sign according to the odrive and
 * torque is pulling force so positive x direction must be pulling direction
 *  2. I'm too scared to change the code now
 */

#pragma once

#ifdef Arduino_h
#include <EEPROM.h>  // save zeros
#endif

#include "../unit_test_simulator/arduino_test_utils.h"
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
      : zero_(zero),
        radius_(radius),
        theta_(zero),
        thetaDot_(0),
        error_(0),
        state_(0) {}

  // Calibration
  void setZero() { zero_ = theta_; }
  void setZero(float zero) { zero_ = zero; }
  float zero() const { return zero_; }
  void saveZero(uint8_t addr) const { EEPROM.put(addr, zero_); }
  void restoreZero(uint8_t addr) { EEPROM.get(addr, zero_); }

  // Getter functions
  float len() const {
    return (theta_ - zero_) * radius_ *
           kTau;  // this has a sign error but too risky to change
  }
  float lenDot() const { return thetaDot_ * radius_ * kTau; }
  // Units of revolutions and rev/s!!!
  float theta() const { return theta_; }
  float thetaDot() const { return thetaDot_; }
  uint32_t error() const { return error_; }
  uint32_t state() const { return state_; }

  // Getter functions with conversions
  float theta(float len) const { return len / radius_ / kTau + zero_; }
  float thetaDot(float lenDot) const { return lenDot / radius_ / kTau; }

  // Check if valid
  bool isThetaValid() const { return theta_.isValid(); }
  bool isThetaDotValid() const { return thetaDot_.isValid(); }
  bool isErrorValid() const { return error_.isValid(); }
  bool isStateValid() const { return state_.isValid(); }
  bool isThetaValid(uint64_t th) const { return theta_.isValid(th); }
  bool isThetaDotValid(uint64_t th) const { return thetaDot_.isValid(th); }
  bool isErrorValid(uint64_t th) const { return error_.isValid(th); }
  bool isStateValid(uint64_t th) const { return state_.isValid(th); }

  // State updates from ODrive
  void setTheta(float theta) { theta_ = theta; }
  void setThetaDot(float thetaDot) { thetaDot_ = thetaDot; }
  void setError(float error) { error_ = error; }
  void setState(float state) { state_ = state; }

 protected:
  // Private variables
  float zero_, radius_;
  Timestamped<float> theta_, thetaDot_;
  Timestamped<uint32_t> error_, state_;
};
