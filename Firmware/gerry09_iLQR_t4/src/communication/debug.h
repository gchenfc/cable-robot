/**
 * @file debug.h
 * @author Gerry Chen
 * @brief This class is for printing/sending/saving debug information.
 */

#pragma once

#include <Metro.h>
#include <Stream.h>

#include "../robot.h"
#include "odrive_can.h"
#include "../controllers/controller_interface.h"

class Debug {
 public:
  Debug(Stream& serial, Robot& robot, ControllerInterface* controller,
        Odrive& odrive)
      : serial_(serial),
        robot_(robot),
        controller_(controller),
        odrive_(odrive) {}

  // Common API
  void setup() {}
  void update() {
    if (print_timer_.check()) {
      for (int i = 0; i < 4; ++i) {
        const Winch& winch = robot_.winches.at(i);
        serial_.printf("%d %d %.2f %.2f\t|\t", winch.error(), winch.state(),
                       winch.len(), winch.lenDot());
      }
      serial_.println();
    }
    readSerial();
  }

 private:
  Stream& serial_;
  Robot& robot_;
  ControllerInterface* controller_;
  Odrive& odrive_;
  Metro print_timer_ = Metro(100);

  void readSerial();
};

namespace human_serial {

bool until(char** buffer_start, char* buffer_end, char delim) {
  *buffer_start = std::find(*buffer_start, buffer_end, delim);
  if (*buffer_start == buffer_end) return false;
  *((*buffer_start)++) =
      0;  // null terminate the number and advance to next one
  return true;
}
template <typename T>
bool parseInt(char** buffer_start, char* buffer_end, char delim, T* value) {
  char* original_start = *buffer_start;
  if (!until(buffer_start, buffer_end, delim)) return false;
  *value = atoi(original_start);
  return true;
}
template <typename T>
bool parseFloat(char** buffer_start, char* buffer_end, char delim, T* value) {
  char* original_start = *buffer_start;
  if (!until(buffer_start, buffer_end, delim)) return false;
  *value = atof(original_start);
  return true;
}

bool parseMsgController(ControllerInterface* controller, char* buffer, int size,
                        Stream& serial) {
  char* parse_cur = buffer;
  char* parse_end = buffer + size;
  uint32_t cmd;
  if (parse_cur[0] != 'g') return false;
  ++parse_cur;
  if (!parseInt(&parse_cur, parse_end, '\n', &cmd)) return false;
  switch (cmd) {
    case 1:
      serial.println("GOT COMMAND TO START TRAJECTORY");
      // controller->startTraj();
      return true;
    default:
      serial.println("Invalid controller command code");
      return false;
  }
}

bool parseMsgCanPassthrough(Odrive& odrive, char* buffer, int size,
                            Stream& serial) {
  char* parse_cur = buffer;
  char* parse_end = buffer + size;
  uint8_t node, cmd;
  if (!parseInt(&parse_cur, parse_end, 'n', &node)) return false;
  if (!parseInt(&parse_cur, parse_end, 'c', &cmd)) return false;

  int32_t i1, i2;
  float f1, f2;
  switch (cmd) {
    case MSG_ODRIVE_ESTOP:
    case MSG_START_ANTICOGGING:
    case MSG_RESET_ODRIVE:
    case MSG_CLEAR_ERRORS:
      odrive.send(node, cmd);
      serial.print("Commanded ");
      serial.println(COMMANDS[cmd]);
      return true;
    case MSG_GET_MOTOR_ERROR:
    case MSG_GET_ENCODER_ERROR:
    case MSG_GET_SENSORLESS_ERROR:
    case MSG_GET_ENCODER_ESTIMATES:
    case MSG_GET_ENCODER_COUNT:
    case MSG_GET_IQ:
    case MSG_GET_SENSORLESS_ESTIMATES:
    case MSG_GET_VBUS_VOLTAGE:
      odrive.send(node, cmd, true);
      serial.print("Requested ");
      serial.println(COMMANDS[cmd]);
      return true;
    case MSG_SET_AXIS_NODE_ID:
    case MSG_SET_AXIS_REQUESTED_STATE:
    case MSG_SET_LINEAR_COUNT:
      if (!parseInt(&parse_cur, parse_end, '\n', &i1)) return false;
      odrive.send(node, cmd, i1);
      break;
    case MSG_SET_AXIS_STARTUP_CONFIG:
      serial.println("Not yet implemented!");
      break;
    case MSG_SET_CONTROLLER_MODES:
      if (!parseInt(&parse_cur, parse_end, ',', &i1)) return false;
      if (!parseInt(&parse_cur, parse_end, '\n', &i2)) return false;
      odrive.send(node, cmd, i1, i2);
      break;
    case MSG_SET_INPUT_POS:
      if (!parseFloat(&parse_cur, parse_end, ',', &f1)) return false;
      if (!parseInt(&parse_cur, parse_end, ',', &i1)) return false;
      if (!parseInt(&parse_cur, parse_end, '\n', &i2)) return false;
      odrive.send(node, cmd, f1, i1, i2);
      break;
    case MSG_SET_INPUT_VEL:
    case MSG_SET_TRAJ_ACCEL_LIMITS:
      if (!parseFloat(&parse_cur, parse_end, ',', &f1)) return false;
      if (!parseFloat(&parse_cur, parse_end, '\n', &f2)) return false;
      odrive.send(node, cmd, f1, f2);
      break;
    case MSG_SET_INPUT_TORQUE:
    case MSG_SET_VEL_LIMIT:
    case MSG_SET_TRAJ_VEL_LIMIT:
    case MSG_SET_TRAJ_INERTIA:
      if (!parseFloat(&parse_cur, parse_end, '\n', &f1)) return false;
      odrive.send(node, cmd, f1);
      break;
    default:
      serial.println("Command not recognized!");
      serial.print(node);
      serial.print('\t');
      serial.println(cmd);
      return false;
  }
  serial.print("Set ");
  serial.println(COMMANDS[cmd]);
  return true;
}

}  // namespace human_serial

void Debug::readSerial() {
  static char buffer[1000];
  static int bufferi = 0;
  while (serial_.available()) {
    char c = serial_.read();
    if (c == ';') c = '\n';
    buffer[bufferi] = c;
    bufferi++;
    if (c == '\n') {
      if ((!human_serial::parseMsgController(controller_, buffer, bufferi,
                                             serial_)) &&
          (!human_serial::parseMsgCanPassthrough(odrive_, buffer, bufferi,
                                                 serial_))) {
        serial_.println("Parse Error");
      };
      bufferi = 0;
    }
  }
}
