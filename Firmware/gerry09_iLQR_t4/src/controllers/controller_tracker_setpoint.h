#pragma once

#include "../communication/ascii_parser.h"
#include "controller_interface.h"
#include "setpoint_interface.h"
#include "tracker_interface.h"

class Odrive;

/**
 * ControllerTrackerSetpoint uses the `Setpoint` and `Tracker` separately.
 */
template <class SetpointClass, class TrackerClass>
class ControllerTrackerSetpoint : public ControllerInterface {
 public:
  ControllerTrackerSetpoint(SetpointClass& setpoint,
                            TrackerClass& tracker)
      : setpoint_(&setpoint), tracker_(&tracker) {}

  static void print_name(Stream& serialOut) {
    serialOut.print("ControllerTrackerSetpoint : ");
    SetpointClass::print_name(serialOut);
    serialOut.print(" + ");
    TrackerClass::print_name(serialOut);
  }

  // Common API
  virtual void setup() override {
    setpoint_->setup();
    tracker_->setup();
  };
  virtual void update() override {
    setpoint_->update();
    tracker_->update();
  };

  virtual bool readSerial(AsciiParser parser, Stream& serialOut) override {
    return setpoint_->readSerial(parser, serialOut) ||
           tracker_->readSerial(parser, serialOut);
  }
  virtual void writeSerial(Stream& serialOut) override {
    setpoint_->writeSerial(serialOut);
    tracker_->writeSerial(serialOut);
  }

  /****************************** Controller API ******************************/
  virtual bool encoderMsgCallback(Odrive* odrive,
                                  uint8_t winchnum) const override {
    return tracker_->encoderMsgCallback(odrive, winchnum);
  }

  // This doesn't appear to be called outside of controller_simple so ignore
  virtual bool setupFor(ControllerState state) { return false; }
  // These don't appear to be called anywhere except debug.h, so we'll just
  // "deprecate" these.
  virtual bool goToStartTraj() override { return false; }
  virtual bool startTraj() override { return false; }
  virtual bool stopTraj() override { return false; }
  virtual bool resetTraj() override { return false; }
  virtual bool setToTrajIndex(uint64_t) override { return false; }
  virtual bool hold() override { return false; }
  // This is called during ESTOP
  virtual bool release() override {  // let go and go to IDLE state
    return tracker_->setState(TrackerClass::State::IDLE);
  };

  /******************************* Data Logging *******************************/
  virtual Vector2 setpointPos() const override {
    return makePair(head<2>(setpoint_->setpointPos()));
  }
  virtual float setpointTheta() const override {
    return std::get<2>(setpoint_->setpointPos());
  }
  virtual Vector2 setpointVel() const override {
    return makePair(head<2>(setpoint_->setpointVel()));
  }
  virtual float setpointThetaDot() const override {
    return std::get<2>(setpoint_->setpointVel());
  }

 protected:
  SetpointClass* setpoint_;
  TrackerClass* tracker_;
};
