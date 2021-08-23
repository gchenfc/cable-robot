#pragma once

#include "state_estimator_interface.h"
#include "../kinematics.h"

class StateEstimatorNaiveFirstOrder : public StateEstimatorInterface {
 public:
  StateEstimatorNaiveFirstOrder(Robot& robot) : kinematics_(robot) {}

  void update() override {
    static float W[4][2];
    last_us_ = micros();
    kinematics_.FK(&last_pos_.first, &last_pos_.second);
    kinematics_.jacobian(W);
    kinematics_.FKv(W, &last_vel_.first, &last_vel_.second);
  }

  std::pair<float, float> pos() const override {
    return last_pos_;
  };
  std::pair<float, float> pos(uint64_t time_us) const override {
    std::pair<float, float> pos_now;
    double dt_s = static_cast<double>(time_us - last_us_) / 1e6;
    pos_now.first = last_pos_.first + last_vel_.first * dt_s
    pos_now.second = last_pos_.second + last_vel_.second * dt_s
    return last_pos_;
  };
  std::pair<float, float> vel() const override {
    return last_vel_;
  };

 private:
  Kinematics kinematics_;
  uint64_t last_us_;
  std::pair<float, float> last_pos_;
  std::pair<float, float> last_vel_;
};
