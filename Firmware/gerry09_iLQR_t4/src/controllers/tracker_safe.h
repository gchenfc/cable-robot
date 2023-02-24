#pragma once

#include "../communication/ascii_parser.h"
#include "tracker_interface.h"
#include "../state_estimators/state_estimator_interface.h"
#include "setpoint_interface.h"

/**
 * TrackerSafe is a wrapper around TrackerInterface that adds safety checks to
 * make sure that, if the `setpoint` calculation goes crazy, it will
 * automatically switch back to GRAVITY_COMPENSATION mode.
 */
class TrackerSafe : public TrackerInterface {
 public:
  TrackerSafe(Robot& robot, Odrive& odrive, SetpointInterface* setpoint,
              StateEstimatorInterface* state_estimator)
      : TrackerInterface(robot, odrive, setpoint),
        state_estimator_(state_estimator) {}

  virtual bool readSerial(AsciiParser parser, Stream& serial_out) override;

  /********************* Safety wrappers around setpoint_ *********************/
  SetpointInterface::X setpointPos();
  SetpointInterface::V setpointVel();
  SetpointInterface::A setpointAcc();

 protected:
  float max_distance_to_setpoint_ = 0.05;
  float max_vel_setpoint_ = 3;
  float max_acc_setpoint_ = 30;

 private:
  StateEstimatorInterface* state_estimator_;
};

bool TrackerSafe::readSerial(AsciiParser parser, Stream& serial_out) {
  if (TrackerInterface::readSerial(parser, serial_out)) return true;
  UNWRAP_PARSE_CHECK(, parser.checkChar(SerialPrefixes::TRACKING));
  UNWRAP_PARSE_CHECK(char cmd, parser.getChar(&cmd));

  float v;
  switch (cmd) {
    // state control
    case TrackingCommands::SET_MAX_DISTANCE_TO_SETPOINT:
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &v));
      serial_out.printf("tracking: max_distance_to_setpoint: %.3f\n", v);
      max_distance_to_setpoint_ = v;
      return true;
    case TrackingCommands::SET_MAX_VEL_SETPOINT:
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &v));
      serial_out.printf("tracking: max_vel_setpoint: %.3f\n", v);
      max_vel_setpoint_ = v;
      return true;
    case TrackingCommands::SET_MAX_ACC_SETPOINT:
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &v));
      serial_out.printf("tracking: max_acc_setpoint: %.3f\n", v);
      max_acc_setpoint_ = v;
      return true;
    default:
      return false;
  }
}

SetpointInterface::X TrackerSafe::setpointPos() {
  // I'm concerned this will suck up a lot of compute time, but whatever
  auto cur_ = state_estimator_->posEst();
  SetpointInterface::X cur = {cur_.first, cur_.second,
                              state_estimator_->thetaEst()};
  SetpointInterface::X des = setpoint_->setpointPos();
  if (norm(cur - des) > max_distance_to_setpoint_) {
    if (state_ == POSITION_CONTROL) {
      setState(GRAVITY_COMP);
    }
    return cur;
  } else {
    return des;
  }
}

SetpointInterface::V TrackerSafe::setpointVel() {
  auto des = setpoint_->setpointVel();
  if (norm(des) > max_vel_setpoint_) {
    if (state_ == POSITION_CONTROL) {
      setState(GRAVITY_COMP);
    }
    return SetpointInterface::V{0, 0, 0};
  } else {
    return des;
  }
}

SetpointInterface::A TrackerSafe::setpointAcc() {
  auto des = setpoint_->setpointAcc();
  if (norm(des) > max_acc_setpoint_) {
    if (state_ == POSITION_CONTROL) {
      setState(GRAVITY_COMP);
    }
    return SetpointInterface::A{0, 0, 0};
  } else {
    return des;
  }
}
