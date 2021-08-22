#pragma once

class ControllerInterface {
 public:
  // Common API
  virtual void setup() = 0;
  virtual void update() = 0;

  // Controller API
  virtual float get_torque_now(uint8_t winch_num) const = 0;
};
