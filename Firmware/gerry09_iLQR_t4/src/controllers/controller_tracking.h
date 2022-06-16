#pragma once

#include "controller_simple.h"

class ControllerTracking : public ControllerSimple {
 public:
  ControllerTracking(const StateEstimatorInterface* state_estimator)
      : ControllerSimple(state_estimator),
        cur_(kWidth / 2, kHeight / 2),
        setpoint_(cur_) {}

  // Datalogging
  std::pair<float, float> setpointVel() const override {
    return std::make_pair(0.0f, 0.0f);
  }

  Vector2 getSetpoint() const {
    return setpoint_;
  }
  void setSetpoint(Vector2 setpoint) {
    setpoint_ = setpoint;
  }

 protected:
  static constexpr float SPEED = 0.5;
  Vector2 cur_, setpoint_;
  Metro setpointUpdateTimer_{1};

  void myUpdate() override {
    if (setpointUpdateTimer_.check()) {
      if (state_ == HOLD_TRAJ_BEGIN) {
        cur_ = state_estimator_->posEst();
        setpoint_ = cur_;
      }
      towards(1e-3 * SPEED,                       //
              cur_.first, cur_.second,            // current
              setpoint_.first, setpoint_.second,  // target
              &cur_.first, &cur_.second);         // new
      clamp(&cur_.first, 0.3, kWidth-0.3);
      clamp(&cur_.second, 0.3, kHeight-0.3);
      clamp(&setpoint_.first, 0.3, kWidth-0.3);
      clamp(&setpoint_.second, 0.3, kHeight-0.3);
    }
  }

  Vector2 desPos(float t) const override {
    return cur_;
  }
};
