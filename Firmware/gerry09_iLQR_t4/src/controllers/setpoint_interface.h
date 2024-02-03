#pragma once

#include <Metro.h>

#include "../communication/ascii_parser.h"
#include "../Vector.h"

/**
 * SetpointInterface defines the common interface that feeds the setpoint
 * pos/vel/acc to a TrackerInterface.
 *
 * SetpointInterface must implement setpointPos, setpointVel, setpointAcc
 * (feedforward), and commands like start, stop, pause, etc.
 */
class SetpointInterface {
 public:
  using X = Vector<3>;  // x, y, theta
  using V = Vector<3>;
  using A = Vector<3>;

  // Status describes whether the setpoint is ready or not
  enum Status {
    UNINITIALIZED,
    INITIALIZING,
    NOMINAL,
    TRACKING_SOFT_ERROR,  // When the tracking error is too large, hold in place
  };

  enum State {
    HOLD,
    RUNNING,
    INTERMEDIATE_TRAVEL,
  };

  /******************************** Common API ********************************/
  virtual void setup(){};         // Called on bootup
  virtual void update(){};        // Called every loop
  virtual bool initialize() = 0;  // Must call this when "UNINITIALIZED" status

  virtual bool readSerial(AsciiParser, Stream&) { return false; }
  virtual void writeSerial(Stream&) {}  // called by Debug

  /**************************** State Control API ****************************/
  virtual bool travel() = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;
  virtual bool pause() = 0;
  virtual bool advanceTo(float time_s) = 0;

  /******************************* Setpoint API *******************************/
  virtual X setpointPos() = 0;
  virtual V setpointVel() = 0;
  virtual A setpointAcc() = 0;

  /*************** Implementations that you don't need to touch ***************/
  Status getStatus() const { return status_; }
  State getState() const { return state_; }

 protected:
  Status status_ = Status::UNINITIALIZED;
  State state_ = State::HOLD;
  Metro update_timer_{10};
};
