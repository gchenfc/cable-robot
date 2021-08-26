#pragma once

class Odrive;

/**
 * ControllerInterface defines the common interface for the callback function
 * `encoderMsgCallback` that `Odrive` calls on encoder messages so that an
 * immediate (realtime) response can be made.
 */
class ControllerInterface {
 public:
  using Vector2 = std::pair<float, float>;
  enum ControllerState {
    IDLE,
    SETUP,
    HOLD_TRAJ_BEGIN,
    RUNNING_TRAJ,
    RUNNING_USER,  // TODO(gerry): this is not supported yet
    HOLDING_BAN_INPUT,
    HOLDING_ALLOW_INPUT,
  };

  // Common API
  virtual void setup() = 0;
  virtual void update() = 0;

  /****************************** Controller API ******************************/

  // Returns true if a CAN message was sent, false otherwise (to know whether or
  // not caller should service the watchdog)
  virtual bool encoderMsgCallback(Odrive* odrive, uint8_t winchnum) const = 0;

  // State transition requests
  virtual bool setupFor(ControllerState state) = 0;  // prep for next state
  virtual bool goToStartTraj() = 0;  // go and stay at the first setpoint
  virtual bool startTraj() = 0;      // start (or resume) trajectory
  virtual bool stopTraj() = 0;       // pause trajectory
  virtual bool resetTraj() = 0;      // reset back to beginning of trajectory
  virtual bool setToTrajIndex(uint64_t) = 0;
  virtual bool hold() = 0;     // rigidly stay at a point
  virtual bool release() = 0;  // let go and go to IDLE state

  /******************************* Data Logging *******************************/
  virtual Vector2 setpointPos() const { return {-1, -1}; }
  virtual Vector2 setpointVel() const { return {0, 0}; }

  /******* Implementations that you don't need to touch *******/
  ControllerState getState() const { return state_; }

 protected:
  ControllerState state_ = IDLE;
};
