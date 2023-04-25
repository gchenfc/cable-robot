/**
 * This controls the paint brush arm over serial.
 * For now, we're just forwarding the serial from the computer to the arm.
 */

#pragma once

#include <Stream.h>
template <typename SerialComputerT>
class Arm {
 public:
  Arm(SerialComputerT& computer, HardwareSerial& arm)
      : computer_(computer), arm_(arm) {}

  // Common API
  void setup() { arm_.begin(baud_); }
  void update() {
    if (baud_ != computer_.baud()) {
      arm_.begin(baud_ = computer_.baud());
    }
    // Forward the arm serial to/from the computer multiple bytes at a time, and
    // ensure that availableForWrite() is enough on both sides.
    static char buf[1000];
    if (arm_.available() && computer_.availableForWrite()) {
      int n = std::min({arm_.available(), computer_.availableForWrite(), 1000});
      arm_.readBytes(buf, n);
      computer_.write(buf, n);
    }
    if (computer_.available() && arm_.availableForWrite()) {
      int n = std::min({computer_.available(), arm_.availableForWrite(), 1000});
      computer_.readBytes(buf, n);
      arm_.write(buf, n);
    }
  }

 private:
  SerialComputerT& computer_;
  HardwareSerial& arm_;
  uint32_t baud_ = 9600;
};
