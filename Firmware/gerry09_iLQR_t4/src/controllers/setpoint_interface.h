#pragma once

#include "../communication/ascii_parser.h"
#include "../Vector.h"

class Odrive;

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

  virtual bool readSerial(AsciiParser parser, Stream& serialOut) {
    return false;
  }
  virtual void writeSerial(Stream& serialOut) {}  // called by Debug

  /**************************** State Control API ****************************/
  // State transition requests
  virtual bool setupFor(ControllerState state) = 0;  // prep for next state
  // TODO: figure out what functions this interface should provide
  virtual bool startIntermediate() = 0;
  virtual bool startAll() = 0;

  virtual bool goToStartTraj() = 0;  // go and stay at the first setpoint
  virtual bool startTraj() = 0;      // start (or resume) trajectory
  virtual bool stopTraj() = 0;       // pause trajectory
  virtual bool resetTraj() = 0;      // reset back to beginning of trajectory
  virtual bool setToTrajIndex(uint64_t) = 0;
  virtual bool hold() = 0;     // rigidly stay at a point
  virtual bool release() = 0;  // let go and go to IDLE state

  /******************************* Setpoint API *******************************/
  virtual X setpointPos() = 0;
  virtual V setpointVel() = 0;
  virtual A setpointAcc() = 0;

  /*************** Implementations that you don't need to touch ***************/
  State getState() const { return state_; }
  State getState() const { return state_; }

 protected:
  Status status_ = INVALID;
  State state_ = HOLD;
};
