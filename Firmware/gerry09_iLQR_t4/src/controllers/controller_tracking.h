#pragma once

#include "controller_simple.h"

#if defined(KLAUS) || defined(DFL)
static constexpr float kSpeed = 1.0;
#endif
#ifdef HYDROPONICS
static constexpr float kSpeed = 0.1;
#endif

class ControllerTracking : public ControllerSimple {
 public:
  ControllerTracking(const StateEstimatorInterface* state_estimator)
      : ControllerSimple(state_estimator),
        cur_(kWidth / 2, kHeight / 2),
        setpoint_(cur_) {}

  Vector2 getSetpoint() const {
    return setpoint_;
  }
  void setSetpoint(Vector2 setpoint) {
    setpoint_ = setpoint;
  }

  bool readSerial(AsciiParser parser, Stream& serialOut) override;

 protected:
  float speed = kSpeed;
  Vector2 cur_, setpoint_;
  float limit_left_ = 0.2, limit_right_ = 0.2, limit_up_ = 0.2,
        limit_down_ = 0.2;
  Metro setpointUpdateTimer_{1};

  void myUpdate() override {
    if ((state_ == HOLD_TRAJ_BEGIN) ||
        ((state_ == RUNNING_TRAJ) && (prev_state_ != RUNNING_TRAJ))) {
      cur_ = state_estimator_->posEst();
      setpoint_ = cur_;
    }
    // if (state_ == RUNNING_TRAJ) {
    //   // For safety reasons, snap the desired position to current estimated
    //   // position just in case.
    //   const Vector2& x = state_estimator_->posEst();
    //   towards(1e-3 * speed,                //
    //           x.first, x.second,           //
    //           cur_.first, cur_.second,     //
    //           &cur_.first, &cur_.second);  //
    // }
    if (setpointUpdateTimer_.check()) {
      towards(1e-3 * speed,                       //
              cur_.first, cur_.second,            // current
              setpoint_.first, setpoint_.second,  // target
              &cur_.first, &cur_.second);         // new
      clamp(&cur_.first, limit_left_, kWidth - limit_right_);
      clamp(&cur_.second, limit_down_, kHeight - limit_up_);
      clamp(&setpoint_.first, limit_left_, kWidth - limit_right_);
      clamp(&setpoint_.second, limit_down_, kHeight - limit_up_);
    }
  }

  Vector2 desPos(float t) const override {
    return cur_;
  }
};

bool ControllerTracking::readSerial(AsciiParser parser, Stream& serialOut) {
  UNWRAP_PARSE_CHECK(,parser.checkChar('t'));
  UNWRAP_PARSE_CHECK(char cmd, parser.getChar(&cmd));

  std::pair<float, float> setpoint = getSetpoint();
  float amt;
  switch (cmd) {
    case 'a':
      UNWRAP_PARSE_CHECK(,parser.parseFloat(',', &setpoint.first));
      UNWRAP_PARSE_CHECK(,parser.parseFloat('\n', &setpoint.second));
      break;
    case 'r':
      UNWRAP_PARSE_CHECK(,parser.parseFloat('\n', &amt));
      setpoint.first += amt;
      break;
    case 'l':
      UNWRAP_PARSE_CHECK(,parser.parseFloat('\n', &amt));
      setpoint.first -= amt;
      break;
    case 'u':
      UNWRAP_PARSE_CHECK(,parser.parseFloat('\n', &amt));
      setpoint.second += amt;
      break;
    case 'd':
      UNWRAP_PARSE_CHECK(,parser.parseFloat('\n', &amt));
      setpoint.second -= amt;
      break;
    case 's':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &speed));
      return true;
    case 'L': {  // limits
      UNWRAP_PARSE_CHECK(char dir, parser.getChar(&dir));
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      switch (dir) {
        case 'u':
          limit_up_ = amt;
          return true;
        case 'd':
          limit_down_ = amt;
          return true;
        case 'l':
          limit_left_ = amt;
          return true;
        case 'r':
          limit_right_ = amt;
          return true;
        default:
          return false;
      }
    }
    default:
      return false;
  }
  setSetpoint(setpoint);
  return true;
}
