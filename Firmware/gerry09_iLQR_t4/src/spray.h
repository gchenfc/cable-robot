/**
 * This controls the spray paint actuator over bluetooth.  This code only
 * handles the bluetooth/Teensy side and not the code for the remote
 * micronotroller physically attached to the spray can + servo.
 */

#pragma once

#include <Stream.h>
class Spray {
 public:
  Spray(HardwareSerial& serial) : serial_(serial), on_(false) {}

  // Common API
  void setup() { serial_.begin(9600); }
  void update() {
    if (send_timer_.check()) {
      serial_.write(on_ ? '1' : '0');
    }
  }

  void changeColor(int color) {
    serial_.write('c');
    serial_.println(color, DEC);
  }
  void setSpray(bool on) { on_ = on; }
  bool spray() const { return on_; }

  void forward_msg(char* buf, int len) {
    serial_.write(buf, len);
  }

 private:
  HardwareSerial& serial_;
  bool on_;
  Metro send_timer_{50};
};
