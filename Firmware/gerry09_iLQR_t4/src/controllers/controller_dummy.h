#pragma once

#include "controller_interface.h"

class ControllerDummy : public ControllerInterface {
 public:
  // Common API
  void setup() override {}
  void update() override {}

  // Controller API
  bool encoderMsgCallback(Odrive* odrive, uint8_t winchnum) const override {
    return false;
  }
};
