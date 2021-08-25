#pragma once

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
