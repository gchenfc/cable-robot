#pragma once

#include <cstring>  // memcpy
#include "controller_simple.h"

#if defined(KLAUS) || defined(DFL)
static constexpr float kSpeed = 1.0;
#endif
#ifdef HYDROPONICS
static constexpr float kSpeed = 0.1;
#endif
static constexpr float kAccel = 1.0;
static constexpr size_t kUpdateInterval_ms = 1;

// class TrajectoryManager {
//  private:

// }

class ControllerTracking : public ControllerSimple {
 public:
  ControllerTracking(const StateEstimatorInterface* state_estimator)
      : ControllerSimple(state_estimator),
        cur_(kWidth / 2, kHeight / 2),
        setpoint_() {
    setpoint_.push(cur_);
  }

  float calcDistance() const {
    static constexpr float DOT_THRESH = cosf(30 * kTau / 360);
    float dist = 0;
    auto tmp_setpoints = setpoint_.view();
    Vector2 prev_setpoint = cur_, setpoint;
    bool skip_next = true;
    float prev_dx[2] = {0, 0};
    while (tmp_setpoints.pop(setpoint)) {
      float dx[2] = {setpoint.first - prev_setpoint.first,
                     setpoint.second - prev_setpoint.second};
      float to_add = normalize(dx);
      if (!skip_next && (dot(dx, prev_dx) < DOT_THRESH)) {
        break;  // treat this essentially as a "new stroke"
      }
      dist += to_add;
      prev_setpoint = setpoint;
      std::memcpy(prev_dx, dx, sizeof(dx));
      // skip_next = to_add < 0.01;
      skip_next = skip_next && (to_add < 0.01);
    }
    return dist;
  }
  bool pushSetpoint(Vector2 setpoint) {
    clamp(&setpoint.first, limit_left_, kWidth - limit_right_);
    clamp(&setpoint.second, limit_down_, kHeight - limit_up_);
    if (setpoint_.empty()) {
      setpoint_distance_ = 0;
    } else {
      const auto& last = setpoint_.back();
      float dx[2] = {setpoint.first - last.first,
                     setpoint.second - last.second};
      float d = norm(dx);
      if (d < 0.001) return false;
      setpoint_distance_ += norm(dx);
    }
    return setpoint_.push(setpoint);
  }
  bool dropSetpoint() {
    Vector2 setpoint;
    if (!setpoint_.pop(setpoint)) return false;
    if (setpoint_.empty()) {
      setpoint_distance_ = 0;
    } else {
      const auto& first = setpoint_.front();
      float dx[2] = {setpoint.first - first.first,
                     setpoint.second - first.second};
      setpoint_distance_ -= norm(dx);
    }
    return true;
  }
  bool translateSetpoint(float dx, float dy) {
    if (setpoint_.empty()) return false;
    setpoint_.front().first += dx;
    setpoint_.front().second += dy;
    return true;
  }

  void updateSetpoint();

  bool readSerial(AsciiParser parser, Stream& serialOut) override;

 protected:
  float speed = kSpeed, accel_ = kAccel, cur_speed_ = 0;
  Vector2 cur_;
  CircularBuffer<Vector2, 1000> setpoint_;
  float setpoint_distance_;
  float limit_left_ = 0.2, limit_right_ = 0.2, limit_up_ = 0.2,
        limit_down_ = 0.2;
  Metro setpointUpdateTimer_{kUpdateInterval_ms};
  ControllerState prev_state_ = IDLE;

  void myUpdate() override {
    if ((state_ == HOLD_TRAJ_BEGIN) ||
        ((state_ == RUNNING_TRAJ) && (prev_state_ != RUNNING_TRAJ))) {
      cur_ = state_estimator_->posEst();
      setpoint_.clear();
      setpoint_distance_ = 0;
      pushSetpoint(cur_);
    }
    // if (state_ == RUNNING_TRAJ) {
    //   // For safety reasons, snap the desired position to current estimated
    //   // position just in case.
    //   const Vector2& x = state_estimator_->posEst();
    //   towards(1e-3 * kUpdateInterval_ms * speed,                //
    //           x.first, x.second,           //
    //           cur_.first, cur_.second,     //
    //           &cur_.first, &cur_.second);  //
    // }
    if (setpointUpdateTimer_.check()) {
      updateSetpoint();
      clamp(&cur_.first, limit_left_, kWidth - limit_right_);
      clamp(&cur_.second, limit_down_, kHeight - limit_up_);
    }
    prev_state_ = state_;
  }

  Vector2 desPos(float t) const override { return cur_; }
  // Vector2 desVel(float t) const override { return cur_speed_; }
};

bool ControllerTracking::readSerial(AsciiParser parser, Stream& serialOut) {
  UNWRAP_PARSE_CHECK(, parser.checkChar('t'));
  UNWRAP_PARSE_CHECK(char cmd, parser.getChar(&cmd));

  std::pair<float, float> setpoint;
  float amt;
  switch (cmd) {
    case 'a':
      UNWRAP_PARSE_CHECK(, parser.parseFloat(',', &setpoint.first));
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &setpoint.second));
      pushSetpoint(setpoint);  // returns false if full
      break;
    case 'r':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      translateSetpoint(amt, 0);
      break;
    case 'l':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      translateSetpoint(-amt, 0);
      break;
    case 'u':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      translateSetpoint(0, amt);
      break;
    case 'd':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      translateSetpoint(0, -amt);
      break;
    case 's':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &speed));
      return true;
    case 'A':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &accel_));
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
  return true;
}

void ControllerTracking::updateSetpoint() {
  // first update speed (calculate total path length)
  // float distance_to_final_setpoint = 0;
  // {  // calculate distance to final setpoint
  //   Vector2 first_setpoint = setpoint_.front();
  //   float dx[2] = {first_setpoint.first - cur_.first,
  //                  first_setpoint.second - cur_.second};
  //   distance_to_final_setpoint = setpoint_distance_ + norm(dx);
  // }
  // distance_to_final_setpoint = max(0.0f, distance_to_final_setpoint);
  float distance_to_final_setpoint = calcDistance();
  // Update speed
  float min_new_speed = cur_speed_ - 1e-3 * kUpdateInterval_ms * accel_;
  // float min_new_speed = 0.0f;
  float max_new_speed = cur_speed_ + 1e-3 * kUpdateInterval_ms * accel_;
  // cur_speed_ = sqrt(2 * accel_ * max(0, distance_to_final_setpoint - 0.1));
  cur_speed_ = sqrt(2 * accel_ * distance_to_final_setpoint);
  clamp(&cur_speed_, min_new_speed, min(max_new_speed, speed));
  if ((millis() % 100) == 0) {
    if (Serial.availableForWrite() > 400) {
      Serial.printf(
          "dist: %.4f,    cur speed: %.4f,   min: %.4f,   max: %.4f,   cur "
          "accel: %.4f\n",  //
          distance_to_final_setpoint, cur_speed_, min_new_speed, max_new_speed,
          accel_);
    }
  }

  // now update setpoint
  float dist_to_go = 1e-3 * kUpdateInterval_ms * cur_speed_;
  while (dist_to_go > 0) {
    const Vector2& setpoint = setpoint_.front();
    float d2 = towards(dist_to_go,                       //
                       cur_.first, cur_.second,          // current
                       setpoint.first, setpoint.second,  // target
                       &cur_.first, &cur_.second);       // new
    dist_to_go -= sqrt(d2);
    if (dist_to_go > 0) {          // reached setpoint
      if (setpoint_.size() > 1) {  // always keep at least 1 setpoint
        dropSetpoint();            // continue to next setpoint
      } else {
        break;  // no more setpoints left - continue to current
      }
    }
  }
}
