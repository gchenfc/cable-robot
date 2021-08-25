#pragma once

class Odrive;

/**
 * ControllerInterface defines the common interface for the callback function
 * `encoderMsgCallback` that `Odrive` calls on encoder messages so that an
 * immediate (realtime) response can be made.
 */
class ControllerInterface {
 public:
  // Common API
  virtual void setup() = 0;
  virtual void update() = 0;

  // Controller API
  // Returns true if a CAN message was sent, false otherwise (to know whether or
  // not caller should service the watchdog)
  virtual bool encoderMsgCallback(Odrive* odrive, uint8_t winchnum) const = 0;
};
