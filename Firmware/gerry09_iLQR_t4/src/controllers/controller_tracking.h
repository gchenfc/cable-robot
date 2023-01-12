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

/// @brief TrajectoryManager handles the mathematical aspects of interpolating
/// and retiming a trajectory.
class TrajectoryManager {
 private:
  using Vector2 = std::pair<float, float>;
  using Color = int;  // -1 is no color, 0-6 are real colors
  using Setpoint = std::pair<Vector2, Color>;
  CircularBuffer<Setpoint, 1000> setpoints_;
  float max_speed_, max_accel_;
  float speed_;
  Vector2 x_;

 public:
  TrajectoryManager(float max_speed, float max_accel, Vector2 current_xhat)
      : max_speed_(max_speed), max_accel_(max_accel), speed_(0.0) {
    reset(current_xhat);
  }

  void reset(Vector2 current_xhat) {
    setpoints_.clear();
    speed_ = 0;
    x_ = current_xhat;
    pushSetpoint(x_);
  }

  void setMaxSpeed(float max_speed) { max_speed_ = max_speed; }
  void setMaxAccel(float max_accel) { max_accel_ = max_accel; }
  bool pushSetpoint(Vector2 x, Color color = -1);
  bool dropSetpoint();
  bool translate(float dx, float dy);

  const Vector2& getCurrentSetpoint() const { return x_; }
  void updateSpeed();     // Updates `speed_`
  void updateSetpoint();  // Updates `x_`

 protected:
  /// @brief Returns the path length from the current position to the next
  /// direction change
  /// @param max If the path length is greater than max, then this function will
  /// just return max.
  float calcPathLength(float max = std::numeric_limits<float>::max()) const;
};

/// @brief ControllerTracking takes setpoints/waypoints as Serial input and
/// updates desPos and desVel accordingly
class ControllerTracking : public ControllerSimple {
 public:
  ControllerTracking(const StateEstimatorInterface* state_estimator)
      : ControllerSimple(state_estimator),
        cur_(kWidth / 2, kHeight / 2),
        trajectory_(kSpeed, kAccel, cur_) {}

  static void print_name(Stream& serial) { serial.print("ControllerTracking"); }

  bool readSerial(AsciiParser parser, Stream& serialOut) override;

 protected:
  Vector2 cur_;
  TrajectoryManager trajectory_;
  float limit_left_ = 0.2, limit_right_ = 0.2, limit_up_ = 0.2,
        limit_down_ = 0.2;
  Metro setpointUpdateTimer_{kUpdateInterval_ms};
  ControllerState prev_state_ = IDLE;

  void myUpdate() override {
    if ((state_ == HOLD_TRAJ_BEGIN) ||
        ((state_ == RUNNING_TRAJ) && (prev_state_ != RUNNING_TRAJ))) {
      trajectory_.reset(state_estimator_->posEst());
    }
    if (setpointUpdateTimer_.check()) {
      trajectory_.updateSpeed();
      trajectory_.updateSetpoint();
      cur_ = trajectory_.getCurrentSetpoint();
      clamp(&cur_.first, limit_left_, kWidth - limit_right_);
      clamp(&cur_.second, limit_down_, kHeight - limit_up_);
    }
    prev_state_ = state_;
  }

  Vector2 desPos(float t) const override { return cur_; }
  Vector2 desVel(float t) const override { return {0, 0}; }  // cur_speed_?
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
      clamp(&setpoint.first, limit_left_, kWidth - limit_right_);
      clamp(&setpoint.second, limit_down_, kHeight - limit_up_);
      trajectory_.pushSetpoint(setpoint);  // returns false if full
      break;
    case 'r':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      trajectory_.translate(amt, 0);
      break;
    case 'l':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      trajectory_.translate(-amt, 0);
      break;
    case 'u':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      trajectory_.translate(0, amt);
      break;
    case 'd':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      trajectory_.translate(0, -amt);
      break;
    case 's':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      trajectory_.setMaxSpeed(amt);
      return true;
    case 'A':
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      trajectory_.setMaxAccel(amt);
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

/******************************************************************************/

bool TrajectoryManager::pushSetpoint(Vector2 setpoint, Color color) {
  const auto& last = setpoints_.back().first;
  float dx[2] = {setpoint.first - last.first, setpoint.second - last.second};
  float d2 = norm2(dx);
  if (d2 < (0.001 * 0.001)) return false;
  return setpoints_.push({setpoint, color});
}
bool TrajectoryManager::dropSetpoint() { return setpoints_.drop(); }
bool TrajectoryManager::translate(float dx, float dy) {
  pushSetpoint(Vector2{x_.first + dx, x_.second + dy});
  return true;
}

void TrajectoryManager::updateSpeed() {
  // Update speed
  float distance_max_velocity = 0.5 * max_speed_ * max_speed_ / max_accel_;
  float distance_to_final_setpoint = calcPathLength(distance_max_velocity);
  float min_new_speed = speed_ - 1e-3 * kUpdateInterval_ms * max_accel_;
  float max_new_speed = speed_ + 1e-3 * kUpdateInterval_ms * max_accel_;
  speed_ = sqrt(2 * max_accel_ * distance_to_final_setpoint);
  clamp(&speed_, min_new_speed, min(max_new_speed, max_speed_));

  // Debug print
  if ((millis() % 100) == 0) {
    if (Serial.availableForWrite() > 400) {
      Serial.printf(
          "dist: %.4f,    cur speed: %.4f,   min: %.4f,   max: %.4f,   cur "
          "accel: %.4f\n",  //
          distance_to_final_setpoint, speed_, min_new_speed, max_new_speed,
          max_accel_);
    }
  }
}

void TrajectoryManager::updateSetpoint() {
  float dist_to_go = 1e-3 * kUpdateInterval_ms * speed_;
  while (dist_to_go > 0) {
    const Setpoint& setpoint = setpoints_.front();
    const Vector2& setpoint_xy = setpoint.first;
    float d2 = towards(dist_to_go,                             //
                       x_.first, x_.second,                    // current
                       setpoint_xy.first, setpoint_xy.second,  // target
                       &x_.first, &x_.second);                 // new
    dist_to_go -= sqrt(d2);
    if (dist_to_go > 0) {           // reached setpoint
      if (setpoints_.size() > 1) {  // always keep at least 1 setpoint
        dropSetpoint();             // continue to next setpoint
      } else {
        break;  // no more setpoints left - continue to current
      }
    }
  }
}

float TrajectoryManager::calcPathLength(float max) const {
  static constexpr float DOT_THRESH = cosf(30 * kTau / 360);
  // Initialize stuff
  float dist = 0;
  auto tmp_setpoints = setpoints_.view();  // make a copy to iterate
  Vector2 prev_setpoint = x_, setpoint;
  Setpoint setpoint_color;
  bool skip_next = true;
  float prev_dx[2] = {0, 0};
  // Loop through all the setpoints, accumulating the distances between them
  while (tmp_setpoints.pop(setpoint_color)) {
    setpoint = setpoint_color.first;
    float dx[2] = {setpoint.first - prev_setpoint.first,
                   setpoint.second - prev_setpoint.second};
    float to_add = normalize(dx);
    if (!skip_next && (dot(dx, prev_dx) < DOT_THRESH)) {
      break;  // If we hit a direction change, we want to come to a full stop
    }
    dist += to_add;
    if (dist >= max) {
      return max;
    }
    // update "previous" variables
    prev_setpoint = setpoint;
    std::memcpy(prev_dx, dx, sizeof(dx));
    skip_next = skip_next && (to_add < 0.01);
  }
  return dist;
}
