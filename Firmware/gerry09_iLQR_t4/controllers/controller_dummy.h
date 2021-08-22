#pragma once

#include "controller_interface.h"

class ControllerDummy : public ControllerInterface {
 public:
  // Common API
  void setup() {}
  void update() {}

  // Controller API
  float get_torque_now(uint8_t winch_num) const override { return 0; };
};
