/**
 * This controls the spray paint actuator over bluetooth.  This code only
 * handles the bluetooth/Teensy side and not the code for the remote
 * micronotroller physically attached to the spray can + servo.
 */

#pragma once

#include <Stream.h>

class Spray {
 public:
  Spray(Stream& serial) {}

  // Common API
  void setup() {}
  void update() {}
};
