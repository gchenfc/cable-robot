#pragma once

#include "../robot.h"

class StateEstimatorInterface {
 public:
  // Common API
  virtual void setup() {}
  virtual void update() {}

  /// posEst and velEst estimate the current pos/vel by extrapolating
  virtual std::pair<float, float> posEst(uint64_t time_us = micros()) const = 0;
  virtual std::pair<float, float> velEst(uint64_t time_us = micros()) const = 0;
  virtual float thetaEst(uint64_t time_us = micros()) const;
  virtual float thetaDotEst(uint64_t time_us = micros()) const;
  /// lastPos and lastVel return the last full estimation of pos/vel
  /// To get a new "full estimation", run estimate() then call these again
  virtual std::pair<float, float> lastPos() const { return last_pos_; }
  virtual std::pair<float, float> lastVel() const { return last_vel_; }

 protected:
  std::pair<float, float> last_pos_;
  std::pair<float, float> last_vel_;
};

float StateEstimatorInterface::thetaEst(__attribute__((unused))
                                        uint64_t time_us) const {
  return 0;
};
float StateEstimatorInterface::thetaDotEst(__attribute__((unused))
                                           uint64_t time_us) const {
  return 0;
};
