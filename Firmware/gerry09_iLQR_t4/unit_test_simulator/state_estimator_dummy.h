#pragma once

#include "../src/state_estimators/state_estimator_interface.h"

class StateEstimatorDummy : public StateEstimatorInterface {
 public:
  std::pair<float, float> posEst(uint64_t /*t_us*/ = micros()) const override {
    return last_pos_;
  }
  std::pair<float, float> velEst(uint64_t /*t_us*/ = micros()) const override {
    return last_vel_;
  }

  void setPos(float x, float y) { last_pos_ = {x, y}; }
  void setVel(float x, float y) { last_vel_ = {x, y}; }
};
