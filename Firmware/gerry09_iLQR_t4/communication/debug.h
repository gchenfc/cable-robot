/**
 * @file debug.h
 * @author Gerry Chen
 * @brief This class is for printing/sending/saving debug information.
 */

#pragma once

#include <Stream.h>

extern Robot robot;

class Debug {
 public:
  Debug(Stream& serial) : serial_(serial) {}

  // Common API
  void setup() {}
  void update() {
    for (int i = 0; i < 4; ++i) {
      const Winch& winch = robot.winches.at(i);
      serial_.printf("%d %d %.2f %.2f\t|\t", winch.error(), winch.state(),
                    winch.len(), winch.lenDot());
    }
    serial_.println();
  }

 private:
  Stream& serial_;
};
