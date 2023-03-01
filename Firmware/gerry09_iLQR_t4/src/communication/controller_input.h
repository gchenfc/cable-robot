/**
 * @file controller_input.h
 * @author Gerry Chen
 * @brief This class is for interpreting tracking controller commands over
 * serial.
 */

#pragma once

#include <Metro.h>

#include "../controllers/controller_tracking.h"

class ControllerInput {
 public:
  ControllerInput(Stream& serial, ControllerTracking* controller)
      : serial_(serial), controller_(controller) {}

  // Common API
  void setup() {}
  void update() {
    readSerial();
  }

 private:
  Stream& serial_;
  ControllerTracking* controller_;

  void readSerial();
};

void ControllerInput::readSerial() {
  static char buffer[1000];
  static int bufferi = 0;
  while (serial_.available()) {
    char c = serial_.read();
    if (c == ';') c = '\n';
    buffer[bufferi] = c;
    bufferi++;
    if (c == '\n') {
      if ((!human_serial::parseMsgRobot(robot_, odrive_, buffer, bufferi,
                                        serial_)) &&
          (!human_serial::parseMsgController(controller_, odrive_, buffer,
                                             bufferi, serial_)) &&
          (!human_serial::parseMsgSpray(spray_, buffer, bufferi, serial_)) &&
          (!human_serial::parseMsgCanPassthrough(odrive_, buffer, bufferi,
                                                 serial_))) {
        serial_.println("Parse Error");
      };
      bufferi = 0;
    }
  }
}
