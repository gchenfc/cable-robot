/**
 * This controls the estop code / behavior.
 */

#pragma once

#include "communication/odrive_can.h"
#include "controllers/controller_interface.h"
#include "spray.h"

#if defined(KLAUS) || defined(DFL) || defined(AIR)
static constexpr uint64_t kEstopDebounceTime_us = 0;
#endif
#ifdef HYDROPONICS
static constexpr uint64_t kEstopDebounceTime_us = 500;
#endif

template <int PIN>
class Estop {
 public:
  Estop(Odrive& odrive, ControllerInterface* controller, Spray& spray)
      : odrive_(odrive), controller_(controller), spray_(spray) {}

  // Common API
  void setup() { pinMode(PIN, INPUT_PULLUP); }
  void update() {
    bool on = digitalRead(PIN);
    if (on) {
      if ((micros() - t_last_off_us_) >= kEstopDebounceTime_us) {
        odrive.send(0, MSG_ODRIVE_ESTOP);
        odrive.send(3, MSG_ODRIVE_ESTOP);
        odrive.send(1, MSG_ODRIVE_ESTOP);
        odrive.send(2, MSG_ODRIVE_ESTOP);
        controller_->release();
        spray_.setSpray(false);
      }
    } else {
      t_last_off_us_ = micros();
    }
  }

 private:
  Odrive& odrive_;
  ControllerInterface* controller_;
  Spray& spray_;
  uint64_t t_last_off_us_ = 0;
};
