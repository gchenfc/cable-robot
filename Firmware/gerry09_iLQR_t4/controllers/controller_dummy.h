#pragma once

#include "controller_interface.h"

class ControllerDummy : public ControllerInterface {
 public:
  // Common API
  void setup() {}
  void update() {}

  // Controller API
  bool encoderMsgCallback(Odrive* odrive, uint8_t winchnum) override {
    return false;
  }
};
