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
   * @param winchi winch index (0-3)
   * @param zero the angle (rad) that ODrive reports when the winch is fully
   * reeled in
   * @param radius the radius of the winch [default: kR]
   */
  explicit Winch(uint8_t winchi)
      : zero_(0),
        radius_(kR),
        lenCorrectionParams_(lenCorrectionParamsAll[winchi]),
        mountPoint_(mountPoints[winchi]),
        theta_(0),
        thetaDot_(0),
        commandedTorque_(0),
        error_(0),
        state_(0) {}

  // Calibration
  void setZero() { zero_ = theta_; }
  void setZero(float zero) { zero_ = zero; }
  float zero() const { return zero_; }
  void setZeroFromCurrentLength(float measured_length) {
    // theta_raw - zero = theta_expected
    setZero(0);
    float theta_expected = theta(measured_length);
    setZero(theta_ - theta_expected);
  }
  void setLenCorrectionParams(float (&lenCorrectionParams)[3]) {
    std::copy(std::begin(lenCorrectionParams), std::end(lenCorrectionParams),
              std::begin(lenCorrectionParams_));
  }
  void setMountPoint(float (&mountPoint)[3]) {
    std::copy(std::begin(mountPoint), std::end(mountPoint),
              std::begin(mountPoint_));
  }

  // Getter functions
  // Units of revolutions and rev/s!!!
  float theta() const { return theta_; }
  float thetaDot() const { return thetaDot_; }
  float torque() const { return commandedTorque_; }
  uint32_t error() const { return error_; }
  uint32_t state() const { return state_; }

  // Getter functions with conversions:
  //    len/lenDot: the "calibrated" lengths
  //    lenRaw/lenDotRaw: the "uncalibrated" lengths
  //    theta/thetaDot: inverse of len/lenDot
  //    thetaFromRaw/thetaDotFromRaw: inverse of lenRaw/lenDotRaw
  float len() const { return lenCorrection(lenRaw(), lenCorrectionParams_); }
  float lenDot() const {
    return lenDotCorrection(lenRaw(), lenDotRaw(), lenCorrectionParams_);
  }
  float lenRaw() const { return -(theta_ - zero_) * radius_ * kTau; }
  float lenDotRaw() const { return -thetaDot_ * radius_ * kTau; }
  float thetaFromRaw(float len) const { return -len / radius_ / kTau + zero_; }
  float thetaDotFromRaw(float lenDot) const { return -lenDot / radius_ / kTau; }
  float theta(float len) const {
    return thetaFromRaw(lenCorrectionInv(len, lenCorrectionParams_));
  }
  float thetaDot(float len, float lenDot) const {
    return thetaDotFromRaw(
        lenDotCorrectionInv(len, lenDot, lenCorrectionParams_));
  }

  // Tension to torque and vice-versa
  //   tau * thetadot = tension * ldot
  //   ldot / thetadot = tau / tension
  //   dl / dtheta = tau / tension
  //   tau = tension * (dl / dtheta)
  float tension() const { return tension_N(torque()); }
  float torque_Nm(float tension_N) const {
    float lraw_H_theta = radius_;
    float l_H_lraw = lenDotCorrection(lenRaw(), 1, lenCorrectionParams_);
    float l_H_theta = l_H_lraw * lraw_H_theta;
    return tension_N * l_H_theta;
  }
  float tension_N(float torque_Nm) const {
    float lraw_H_theta = radius_;
    float l_H_lraw = lenDotCorrection(lenRaw(), 1, lenCorrectionParams_);
    float l_H_theta = l_H_lraw * lraw_H_theta;
    return torque_Nm / l_H_theta;
  }

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
  void setTorque(float torque) { commandedTorque_ = torque; }
  void setError(float error) { error_ = error; }
  void setState(float state) { state_ = state; }

  // Saving/Restoring parameters
  void saveZero(uint8_t addr) const { EEPROM.put(addr, zero_); }
  void restoreZero(uint8_t addr) { EEPROM.get(addr, zero_); }
  void saveLenCorrectionParams(uint8_t addr) const {
    EEPROM.put(addr + sizeof(float), lenCorrectionParams_);
  }
  void restoreLenCorrectionParams(uint8_t addr) {
    EEPROM.get(addr + sizeof(float), lenCorrectionParams_);
  }
  void saveMountPoint(uint8_t addr) const {
    EEPROM.put(addr + sizeof(float) + sizeof(lenCorrectionParams_),
               mountPoint_);
  }
  void restoreMountPoint(uint8_t addr) {
    EEPROM.get(addr + sizeof(float) + sizeof(lenCorrectionParams_),
               mountPoint_);
  }
  void saveAll(uint8_t addr) const {
    saveZero(addr);
    saveLenCorrectionParams(addr);
    saveMountPoint(addr);
  }
  void restoreAll(uint8_t addr) {
    restoreZero(addr);
    restoreLenCorrectionParams(addr);
    restoreMountPoint(addr);
  }

 protected:
  // Private variables
  float zero_, radius_;
  float (&lenCorrectionParams_)[3], (&mountPoint_)[2];  // these are references!
  Timestamped<float> theta_, thetaDot_, commandedTorque_;
  Timestamped<uint32_t> error_, state_;

 public:
  static constexpr size_t STORAGE_SIZE =
      sizeof(float) + sizeof(lenCorrectionParams_) + sizeof(mountPoint_);
};
