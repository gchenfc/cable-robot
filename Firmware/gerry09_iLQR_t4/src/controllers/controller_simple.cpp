#include "controller_simple.h"
#include "../communication/odrive_can.h"
#include "../utils.h"

using Vector2 = std::pair<float, float>;

namespace {
Vector2 calcDesPos(float t) {
  return {0.5 * cosf(t * M_PI / 5),  //
          0.5 * sinf(t * M_PI / 5)};
}
float calcTorque(Vector2 pos, Vector2 vel, Vector2 des_pos, Vector2 des_vel,
                 uint8_t winchnum) {
  float error[2], bPa[2];
  error[0] = des_pos.first - pos.first;
  error[1] = des_pos.second - pos.second;
  bPa[0] = kMountPoints[winchnum][0] - pos.first;
  bPa[1] = kMountPoints[winchnum][1] - pos.second;
  normalize<2>(bPa);
  return dot<2>(error, bPa) * 2.0;
}
}  // namespace

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
      tstart_us_ = micros();
      for (int i = 0; i < 4; ++i) {
        odrive.send(0, MSG_SET_INPUT_TORQUE, 0.0f);
        odrive.send(0, MSG_SET_CONTROLLER_MODES, static_cast<int32_t>(3),
                    static_cast<int32_t>(1));
      }
      next_state = RUNNING_TRAJ;
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
      Vector2 des_pos =
          calcDesPos(static_cast<float>(micros() - tstart_us_) / 1e6);
      Vector2 des_vel = {0, 0};
      float torque =
          calcTorque(state_estimator_->pos(), state_estimator_->vel(),  //
                     des_pos, des_vel, winchnum);
      clamp(&torque, -1.0, 1.0);
      return odrive->send(winchnum, MSG_SET_INPUT_TORQUE, torque);
    }
  }
  return false;
}

bool ControllerSimple::startTraj() {
  if ((state_ == IDLE) || (state_ == HOLDING_BAN_INPUT) ||
      (state_ == HOLDING_ALLOW_INPUT)) {
    state_ = SETUP;
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
bool ControllerSimple::setToTrajIndex(uint64_t index) { return true; }
bool ControllerSimple::hold() {
  state_ = HOLDING_BAN_INPUT;
  return true;
}
bool ControllerSimple::release() {
  state_ = IDLE;
  return true;
}
