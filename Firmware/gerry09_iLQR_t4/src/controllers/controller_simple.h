#pragma once

#include <Metro.h>

#include "controller_interface.h"
#include "../state_estimators/state_estimator_interface.h"
#include "../communication/odrive_can.h"
#include "../utils.h"
#include "../communication/can_simple.h"

extern Odrive odrive;

class ControllerSimple : public ControllerInterface {
 public:
  ControllerSimple(const StateEstimatorInterface* state_estimator)
      : state_estimator_(state_estimator) {}

  void setup() override;
  void update() override;

  bool encoderMsgCallback(Odrive* odrive, uint8_t winchnum) const override;

  // State transition requests
  bool setupFor(ControllerState state) override;
  bool startTraj() override;
  bool stopTraj() override;
  bool resetTraj() override;
  bool setToTrajIndex(uint64_t index) override;
  bool hold() override;
  bool release() override;

  // Datalogging
  Vector2 setpointPos() const override {
    return (state_ == RUNNING_TRAJ) ? desPos((micros() - tstart_us_) * 1e-6)
                                    : std::make_pair(0.0f, 0.0f);
  }

 private:
  using Vector2 = std::pair<float, float>;

  Metro updateTimer{50};
  const StateEstimatorInterface* state_estimator_;
  uint64_t tstart_us_;

  virtual Vector2 desPos(float t) const;
  virtual float calcTorque(Vector2 pos, Vector2 vel, Vector2 des_pos,
                           Vector2 des_vel, uint8_t winchnum) const;
};

/************* KEY FUNCTIONS **************/
ControllerSimple::Vector2 ControllerSimple::desPos(float t) const {
  return {1.5 + 0.5 * cosf(t * M_PI / 5),  //
          1.1 + 0.5 * sinf(t * M_PI / 5)};
}
float ControllerSimple::calcTorque(Vector2 pos, Vector2 vel, Vector2 des_pos,
                                   Vector2 des_vel, uint8_t winchnum) const {
  (void)(vel);
  (void)(des_vel);
  float error[2], bPa[2];
  error[0] = des_pos.first - pos.first;
  error[1] = des_pos.second - pos.second;
  bPa[0] = kMountPoints[winchnum][0] - pos.first;
  bPa[1] = kMountPoints[winchnum][1] - pos.second;
  normalize<2>(bPa);
  // Serial.printf("%.2f %.2f %.2f %.2f %.2f %.2f\n",   //
  //               error[0], error[1], bPa[0], bPa[1],  //
  //               dot<2>(error, bPa), dot<2>(error, bPa) * 2.0f + 0.7f);
  static constexpr float gain = 75.0f;
  static constexpr float middle = 0.5f;
  return dot<2>(error, bPa) * gain + middle;
}
/************* END KEY FUNCTIONS **************/

void ControllerSimple::setup() { state_ = IDLE; }
void ControllerSimple::update() {
  if (!updateTimer.check()) {
    return;
  }
  ControllerState next_state = state_;
  switch (state_) {
    case IDLE:
      break;
    case SETUP:
      next_state = IDLE;  // unused state
      break;
    case RUNNING_TRAJ:
      break;
    case RUNNING_USER:
      next_state = IDLE;
      break;
    case HOLDING_BAN_INPUT:
      next_state = HOLDING_ALLOW_INPUT;
      break;
    case HOLDING_ALLOW_INPUT:
      break;
  }
  state_ = next_state;
}

bool ControllerSimple::encoderMsgCallback(Odrive* odrive,
                                          uint8_t winchnum) const {
  switch (state_) {
    case IDLE:
    case RUNNING_USER:
      return odrive->send(winchnum, MSG_SET_INPUT_TORQUE, 0.0f);
    case SETUP:
    case HOLDING_BAN_INPUT:
    case HOLDING_ALLOW_INPUT:
      return odrive->send(winchnum, MSG_SET_INPUT_TORQUE, 0.2f);
    case RUNNING_TRAJ: {
      Vector2 des_pos = desPos(static_cast<float>(micros() - tstart_us_) / 1e6);
      Vector2 des_vel = {0, 0};
      float torque =
          calcTorque(state_estimator_->posEst(), state_estimator_->velEst(),  //
                     des_pos, des_vel, winchnum);
      clamp(&torque, -0.1, 1.2);
      return odrive->send(winchnum, MSG_SET_INPUT_TORQUE, torque);
    }
  }
  return false;
}

bool ControllerSimple::setupFor(ControllerState state) {
  for (int i = 0; i < 4; ++i) {
    switch (state) {
      case HOLDING_BAN_INPUT:
      case HOLDING_ALLOW_INPUT:
        odrive.send(i, MSG_SET_INPUT_TORQUE, 0.2f);
        break;
      case RUNNING_TRAJ:
        odrive.send(i, MSG_SET_INPUT_TORQUE, 0.0f);
        break;
      default:
        odrive.send(i, MSG_SET_INPUT_TORQUE, 0.0f);
    }
    odrive.send(i, MSG_SET_CONTROLLER_MODES, static_cast<int32_t>(1),
                static_cast<int32_t>(1));
  }
  return true;
}
bool ControllerSimple::startTraj() {
  if ((state_ == IDLE) || (state_ == HOLDING_BAN_INPUT) ||
      (state_ == HOLDING_ALLOW_INPUT)) {
    tstart_us_ = micros();
    setupFor(RUNNING_TRAJ);
    state_ = RUNNING_TRAJ;
    return true;
  }
  return false;
}
bool ControllerSimple::stopTraj() {
  if (state_ == RUNNING_TRAJ) {
    state_ = HOLDING_BAN_INPUT;
    return true;
  }
  return false;
}
bool ControllerSimple::resetTraj() { return true; }
bool ControllerSimple::setToTrajIndex(uint64_t index) {
  (void)(index);  // unused
  return true;
}
bool ControllerSimple::hold() {
  tstart_us_ = micros();
  setupFor(HOLDING_ALLOW_INPUT);
  state_ = HOLDING_ALLOW_INPUT;
  return true;
}
bool ControllerSimple::release() {
  setupFor(IDLE);
  state_ = IDLE;
  return true;
}
