/**
 * This controls the estop code / behavior.
 */

#pragma once

#include "communication/odrive_can.h"
#include "controllers/controller_interface.h"
#include "spray.h"

template <int PIN>
class Estop {
 public:
  Estop(Odrive& odrive, ControllerInterface* controller, Spray& spray)
      : odrive_(odrive), controller_(controller), spray_(spray) {}

  // Common API
  void setup() { pinMode(PIN, INPUT_PULLUP); }
  void update() {
    if (digitalRead(PIN)) {
      // Serial.println("ESTOP PRESSED!!!");
      odrive.send(0, MSG_ODRIVE_ESTOP);
      odrive.send(3, MSG_ODRIVE_ESTOP);
      odrive.send(1, MSG_ODRIVE_ESTOP);
      odrive.send(2, MSG_ODRIVE_ESTOP);
      controller_->release();
      spray_.setSpray(false);
    }
  }

 private:
  Odrive& odrive_;
  ControllerInterface* controller_;
  Spray& spray_;
};
