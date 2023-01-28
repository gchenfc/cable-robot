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
      : state_estimator_(state_estimator),
        tstart_us_(0),
        tpause_us_(0),
        tmin_us_(0) {}

  void setup() override;
  void update() override;

  bool encoderMsgCallback(Odrive* odrive, uint8_t winchnum) const override;

  // State transition requests
  bool setupFor(ControllerState state) override;
  bool goToStartTraj() override;
  bool startTraj() override;
  bool stopTraj() override;
  bool resetTraj() override;
  bool setToTrajIndex(uint64_t index) override;
  bool hold() override;
  bool release() override;

  // Datalogging
  Vector2 setpointPos() const override { return desPos(trajTime_s()); }
  float setpointTheta() const override { return desTheta(trajTime_s()); }
  Vector2 setpointVel() const override { return desVel(trajTime_s()); }
  float setpointThetaDot() const override { return desThetaDot(trajTime_s()); }

  // Read serial
  bool readSerial(AsciiParser parser, Stream& serialOut) override {
    if (ControllerInterface::readSerial(parser, serialOut)) return true;

    UNWRAP_PARSE_CHECK(, parser.checkChar('g'));
    UNWRAP_PARSE_CHECK(uint32_t cmd, parser.parseInt(&cmd));
    switch (cmd) {
      case 10: {  // Set hold torque
        UNWRAP_PARSE_CHECK(float t, parser.parseFloat('\n', &t));
        hold_torque_Nm = t;
        serialOut.printf("Hold torque set to %f\n", hold_torque_Nm);
        return true;
      }
    }
    return false;
  }

 protected:
  using Vector2 = std::pair<float, float>;

  Metro updateTimer{10};
  const StateEstimatorInterface* state_estimator_;
  uint64_t tstart_us_, tpause_us_, tmin_us_;
  float hold_torque_Nm = 0.15f;

  // To override
  virtual Vector2 desPos(float t) const;
  virtual float desTheta(float t) const { return 0.0; }
  virtual Vector2 desVel(float t) const { return std::make_pair(0.0f, 0.0f); }
  virtual float desThetaDot(float t) const { return 0.0; }
  virtual float calcTorque(float t, uint8_t winchnum) const;
  // End to override
  virtual void updatePaint(float t) {}
  float trajTime_s() const {
    switch (state_) {
      case IDLE:
      case RUNNING_USER:
      case SETUP:
        return 0.0;
      case HOLDING_BAN_INPUT:
      case HOLDING_ALLOW_INPUT:
      case HOLD_TRAJ_BEGIN:
        return static_cast<float>(tpause_us_) / 1e6f;
      case RUNNING_TRAJ: {
        // note: be careful with kSprayDelay_s
        uint64_t t_us =
            std::max<uint64_t>(micros(), tmin_us_ + tstart_us_) - tstart_us_;
        return static_cast<float>(t_us) / 1e6;
      }
      default:
        return 0.0;  // error
    }
  }
  virtual void myUpdate() {};
};

/************* KEY FUNCTIONS **************/
ControllerSimple::Vector2 ControllerSimple::desPos(float t) const {
  return {1.0 + 0.35 * cosf(t * M_PI / 5),  //
          0.9 + 0.35 * sinf(t * M_PI / 5)};
}
float ControllerSimple::calcTorque(float t, uint8_t winchnum) const {
  const Vector2& pos = state_estimator_->posEst();
  const Vector2& des_pos = desPos(t);

  float error[2], bPa[2];
  error[0] = des_pos.first - pos.first;
  error[1] = des_pos.second - pos.second;
  bPa[0] = mountPoints[winchnum][0] - pos.first;
  bPa[1] = mountPoints[winchnum][1] - pos.second;
  normalize<2>(bPa);
  // Serial.printf("%.2f %.2f %.2f %.2f %.2f %.2f\n",   //
  //               error[0], error[1], bPa[0], bPa[1],  //
  //               dot<2>(error, bPa), dot<2>(error, bPa) * 2.0f + 0.7f);
  static constexpr float gain = 20.0f;
  static constexpr float middle = 0.7f;
  float torque = dot<2>(error, bPa) * gain + middle;
  clamp(&torque, 0.0, 1.4);
  return torque;
}
/************* END KEY FUNCTIONS **************/

void ControllerSimple::setup() { state_ = IDLE; }
void ControllerSimple::update() {
  myUpdate();
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
    case HOLD_TRAJ_BEGIN:
      break;
    case RUNNING_TRAJ:
      updatePaint(trajTime_s() + kSprayDelay_s);
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
      // return odrive->send(winchnum, MSG_SET_INPUT_TORQUE, 0.0f);
      return odrive->send(winchnum, MSG_CO_HEARTBEAT_CMD & 0b11111);
    case SETUP:
    case HOLDING_BAN_INPUT:
    case HOLDING_ALLOW_INPUT:
      return odrive->send(winchnum, MSG_SET_INPUT_TORQUE, hold_torque_Nm);
    case HOLD_TRAJ_BEGIN: {
      float torque = calcTorque(tpause_us_ / static_cast<float>(1e6f), winchnum);
      return odrive->send(winchnum, MSG_SET_INPUT_TORQUE, torque);
    }
    case RUNNING_TRAJ: {
      float torque = calcTorque(trajTime_s(), winchnum);
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
        updatePaint(-1);
        odrive.send(i, MSG_SET_INPUT_TORQUE, 0.2f);
        break;
      case HOLD_TRAJ_BEGIN:
        updatePaint(-1);
        // no break
      case RUNNING_TRAJ:
        odrive.send(i, MSG_SET_INPUT_TORQUE, 0.0f);
        break;
      default:
        updatePaint(-1);
        odrive.send(i, MSG_SET_INPUT_TORQUE, 0.0f);
    }
    odrive.send(i, MSG_SET_CONTROLLER_MODES, static_cast<int32_t>(1),
                static_cast<int32_t>(1));
  }
  return true;
}
bool ControllerSimple::goToStartTraj() {
  if ((state_ == IDLE) || (state_ == HOLDING_BAN_INPUT) ||
      (state_ == HOLDING_ALLOW_INPUT)) {
    state_ = HOLD_TRAJ_BEGIN;
    return true;
  }
  return false;
}
bool ControllerSimple::startTraj() {
  if (state_ == HOLD_TRAJ_BEGIN) {
    // tpause_us_ = 0; // TODO(gerry): remove this later
    tstart_us_ = micros() - tpause_us_ + (kSprayDelay_s * 1e6);
    setupFor(RUNNING_TRAJ);
    state_ = RUNNING_TRAJ;
    return true;
  }
  return false;
}
bool ControllerSimple::stopTraj() {
  if ((state_ == RUNNING_TRAJ) || (state_ == HOLD_TRAJ_BEGIN)) {
    tpause_us_ = trajTime_s() * 1e6;
    tmin_us_ = tpause_us_;
    updatePaint(-1);
    state_ = HOLD_TRAJ_BEGIN;
    return true;
  }
  return false;
}
bool ControllerSimple::resetTraj() {
  tpause_us_ = 0;
  tmin_us_ = 0;
  return true;
}
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
