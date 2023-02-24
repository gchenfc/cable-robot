#pragma once

#include <Metro.h>

#include "../communication/ascii_parser.h"
#include "../communication/cdpr_serial_commands.h"
#include "setpoint_interface.h"
#include "../communication/odrive_can.h"
#include "../communication/can_simple.h"
#include "../robot.h"

extern Odrive odrive;  // Stupid hack - used to send the very initial torque

/**
 * TrackerInterface defines the common interface for a controller that, given an
 * setpoint pos/vel/acc and estimated pos/vel, computes the control inputs
 * (tensions) needed to send to the motor controllers.
 *
 * The most important function is `calcTension_N` that is called in the hot loop
 * to compute the cable tensions (control input) so that an immediate (realtime)
 * response can be made.  Absolutely must be realtime - must return in <100us
 * roughly.
 */
class TrackerInterface {
 public:
  enum State {
    IDLE,
    GRAVITY_COMP,  // this isn't real gravity compensation, but it's "pushable"
    POSITION_CONTROL,
  };

  TrackerInterface(Robot& robot, SetpointInterface* setpoint)
      : robot_(robot), setpoint_(setpoint) {}

  /******************************** Common API ********************************/
  virtual void setup(){};
  virtual void update() {
    if (!update_timer_.check()) return;
    if (state_ == POSITION_CONTROL) {
      if (setpoint_->getStatus() != SetpointInterface::Status::NOMINAL) {
        setState(GRAVITY_COMP);  // kill the controller
      }
    }
  };

  virtual bool readSerial(AsciiParser parser, Stream& serial_out);
  virtual void writeSerial(Stream&) {}

  /****************************** Controller API ******************************/
  // Returns true if a CAN message was sent, false otherwise (to know whether or
  // not caller should service the watchdog)
  bool encoderMsgCallback(Odrive* odrive, uint8_t winchnum);

  /****************************** IMPLEMENT THIS ******************************/
  virtual float calcTension_N(uint8_t winchnum) = 0;
  virtual bool initialize() { return true; }  // e.g. init PID state (optional)

  /*************** Implementations that you don't need to touch ***************/
  State getState() const { return state_; }
  bool setState(const State& state);

 protected:
  State state_ = IDLE;
  Metro update_timer_{10};
  Robot& robot_;

  // Tunable parameters
  float hold_torque_Nm_ = 0.2;

 private:
  SetpointInterface* setpoint_;  // Use safety wrappers in TrackerSafe
  friend class TrackerSafe;
};

bool TrackerInterface::readSerial(AsciiParser parser, Stream& serial_out) {
  UNWRAP_PARSE_CHECK(, parser.checkChar(SerialPrefixes::TRACKING));
  UNWRAP_PARSE_CHECK(char cmd, parser.getChar(&cmd));

  switch (cmd) {
    // state control
    case TrackingCommands::IDLE:
      serial_out.println("tracking: IDLE");
      setState(State::IDLE);
      return true;
    case TrackingCommands::GRAVITY_COMP:
      serial_out.println("tracking: GRAVITY_COMP");
      setState(State::GRAVITY_COMP);
      return true;
    case TrackingCommands::POSITION_CONTROL:
      serial_out.println("tracking: POSITION_CONTROL");
      setState(State::POSITION_CONTROL);
      return true;
    case TrackingCommands::SET_HOLDING_TORQUE: {
      UNWRAP_PARSE_CHECK(float torque, parser.parseFloat('\n', &torque));
      hold_torque_Nm_ = torque;
      serial_out.printf("tracking: set_hold_torque: %.3f\n", hold_torque_Nm_);
      return true;
    }
    case SetpointCommands::POLL_STATUS:
      serial_out.printf("tracking: state: %d\n", state_);
      return true;
    default:
      return false;
  }
}

bool TrackerInterface::setState(const State& state) {
  // First validate transition, and change setpoint states if necessary
  switch (state) {
    case IDLE:
      // don't check truthy-ness of setpoint->pause(), always allow IDLE
      if (state_ == POSITION_CONTROL) setpoint_->pause();
      break;
    case GRAVITY_COMP:
      if (state_ == POSITION_CONTROL) {
        if (!setpoint_->pause()) return false;
      }
      break;
    case POSITION_CONTROL:
      if (setpoint_->initialize() && initialize()) break;  // continue
      return false;
    default:
      return false;
  }

  // Now set the initial motor torques
  for (int i = 0; i < 3; i++) {
    switch (state) {
      case IDLE:
        odrive.send(i, MSG_SET_INPUT_TORQUE, 0.0f);
        break;
      case GRAVITY_COMP:
        odrive.send(i, MSG_SET_INPUT_TORQUE, 0.2f);
        break;
      case POSITION_CONTROL:
        odrive.send(i, MSG_SET_INPUT_TORQUE, 0.0f);
        break;
      default:
        return false;
    }
    odrive.send(i, MSG_SET_CONTROLLER_MODES, static_cast<int32_t>(1),
                static_cast<int32_t>(1));
  }

  state_ = state;
  return true;
}

bool TrackerInterface::encoderMsgCallback(Odrive* odrive, uint8_t winchnum) {
  switch (state_) {
    case State::IDLE:
      // return odrive->send(winchnum, MSG_SET_INPUT_TORQUE, 0.0f);
      return odrive->send(winchnum, MSG_CO_HEARTBEAT_CMD & 0b11111);
    case State::GRAVITY_COMP:
      return odrive->send(winchnum, MSG_SET_INPUT_TORQUE, hold_torque_Nm_);
    case State::POSITION_CONTROL: {
      float tension_N = calcTension_N(winchnum);
      float torque_Nm = robot_.winches.at(winchnum).torque_Nm(tension_N);
      return odrive->send(winchnum, MSG_SET_INPUT_TORQUE, torque_Nm);
    }
  }
  return false;
};
