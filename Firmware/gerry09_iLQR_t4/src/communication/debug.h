/**
 * @file debug.h
 * @author Gerry Chen
 * @brief This class is for printing/sending/saving debug information.
 */

#pragma once

#include <Metro.h>

#include "../controllers/controller_interface.h"
#include "../controllers/controller_simple.h"
#include "../controllers/controller_tracking.h"
#include "../robot.h"
#include "../state_estimators/state_estimator_interface.h"
#include "odrive_can.h"

class Debug {
 public:
  Debug(Stream& serial, Robot& robot, ControllerInterface* controller,
        StateEstimatorInterface* estimator, Odrive& odrive, Spray& spray,
        bool (*custom_callback)(char* buffer, int size))
      : serial_(serial),
        robot_(robot),
        controller_(controller),
        estimator_(estimator),
        odrive_(odrive),
        spray_(spray),
        custom_callback_(custom_callback) {}

  // Common API
  void setup() {}
  void update() {
    if (print_timer_.check()) {
      auto est_pos = estimator_->posEst();
      auto des_pos = controller_->setpointPos();
      serial_.printf("%d: %.4f %.4f - %.4f %.4f\t|\t", controller_->getState(),
                     est_pos.first, est_pos.second,  //
                     des_pos.first, des_pos.second);
      for (int i = 0; i < 4; ++i) {
        const Winch& winch = robot_.winches.at(i);
        serial_.printf("%d %d %.4f %.4f\t|\t",  //
                       winch.error(), winch.state(), winch.len(),
                       winch.lenDot());
      }
      serial_.println(spray_.spray());
    }
    readSerial();
  }

 private:
  Stream& serial_;
  Robot& robot_;
  ControllerInterface* controller_;
  StateEstimatorInterface* estimator_;
  Odrive& odrive_;
  Spray& spray_;
  bool (*custom_callback_)(char* buffer, int size);
  Metro print_timer_ = Metro(10);

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

bool parseMsgRobot(Robot& robot, Odrive& odrive, char* buffer, int size,
                   Stream& serial) {
  if (size == 0) return false;
  char* parse_cur = buffer;
  char* parse_end = buffer + size;
  uint32_t cmd;
  if (parse_cur[0] != 'c') return false;
  ++parse_cur;
  if (!parseInt(&parse_cur, parse_end, '\n', &cmd)) return false;

  switch (cmd) {
    case 0:
    case 1:
    case 2:
    case 3: {
      if (robot.winches.at(cmd).state() != 1) {
        serial.printf(
            "\n\nERROR: Winch %d is not in IDLE state - cannot calibrate\n\n");
        return true;
      }
      serial.printf("Calibrating winch %d...\n", cmd);
      odrive.send<int32_t>(cmd, MSG_SET_AXIS_REQUESTED_STATE, 3);
      return true;
    }
    case 4: {
      serial.println("Calibrating all winches...");
      for (int i = 0; i < 4; ++i) {
        if (robot.winches.at(i).state() != 1) {
          serial.printf(
              "\n\nERROR: Winch %d is not in IDLE state - cannot calibrate, "
              "skipping\n\n");
        } else {
          odrive.send<int32_t>(i, MSG_SET_AXIS_REQUESTED_STATE, 3);
          serial.printf("Calibrating winch %d...\n", i);
        }
      }
      return true;
    }
    case 10:
    case 11:
    case 12:
    case 13: {
      uint8_t winchi = cmd - 10;
      serial.printf("Setting winch %d to zero and saving to EEPROM!\n", winchi);
      robot.setZero(winchi);
      robot.saveZero(winchi);
      serial.printf("Zeros:\nfloat kZeros[4] = {%.3f, %.3f, %.3f, %.3f};\n",
                    robot.zero(0), robot.zero(1), robot.zero(2), robot.zero(3));
      return true;
    }
    case 14: {
      serial.println("Setting all winches to zero and saving to EEPROM!");
      robot.setZeroAll();
      robot.saveZeros();
      serial.printf("Zeros:\nfloat kZeros[4] = {%.3f, %.3f, %.3f, %.3f};\n",
                    robot.zero(0), robot.zero(1), robot.zero(2), robot.zero(3));
      return true;
    }
    case 15: {
      serial.printf("Zeros:\nfloat kZeros[4] = {%.3f, %.3f, %.3f, %.3f};\n",
                    robot.zero(0), robot.zero(1), robot.zero(2), robot.zero(3));
      return true;
    }
    case 20:
    case 21:
    case 22:
    case 23: {
      uint8_t winchi = cmd - 20;
      serial.printf("Restoring winch %d zero from EEPROM!\n", winchi);
      robot.restoreZero(winchi);
      serial.printf("New zeros:\nfloat kZeros[4] = {%.3f, %.3f, %.3f, %.3f};\n",
                    robot.zero(0), robot.zero(1), robot.zero(2), robot.zero(3));
      return true;
    }
    case 24: {
      serial.println("Restoring all winch zeros from EEPROM!");
      robot.restoreZeros();
      serial.printf("New zeros:\nfloat kZeros[4] = {%.3f, %.3f, %.3f, %.3f};\n",
                    robot.zero(0), robot.zero(1), robot.zero(2), robot.zero(3));
      return true;
    }
    default:
      serial.println("\n\nInvalid calibration command code\n\n");
      return false;
  }
}

bool parseMsgController(ControllerInterface* controller, Odrive& odrive,
                        char* buffer, int size, Stream& serial) {
  if (size == 0) return false;
  char* parse_cur = buffer;
  char* parse_end = buffer + size;
  uint32_t cmd;
  if (parse_cur[0] != 'g') return false;
  ++parse_cur;
  if (!parseInt(&parse_cur, parse_end, '\n', &cmd)) return false;

  switch (cmd) {
    case 0:
      serial.println("CLEAR ERRORS");
      odrive.send(0, MSG_CLEAR_ERRORS);
      odrive.send(3, MSG_CLEAR_ERRORS);
      return true;
    case 1:
      serial.println("GO TO START TRAJECTORY");
      controller->goToStartTraj();
      return true;
    case 2:
      serial.println("START TRAJECTORY");
      controller->startTraj();
      return true;
    case 3:
      serial.println("STOP TRAJECTORY");
      controller->stopTraj();
      return true;
    case 4:
      serial.println("RESET TRAJ");
      controller->resetTraj();
      return true;
    case 5:
      serial.println("SET TO TRAJ INDEX");
      controller->setToTrajIndex(0);  // TODO(gerry): parse another number
      return true;
    case 6:
      serial.println("HOLD");
      controller->hold();
      return true;
    case 7:
      serial.println("RELEASE");
      controller->release();
      return true;
    case 8:
      serial.println("CLOSED LOOP CONTROL");
      for (int i = 0; i < 4; ++i) {
        odrive.send<int32_t>(i, MSG_SET_AXIS_REQUESTED_STATE, 8);
      }
      return true;
    default:
      serial.println("\n\nInvalid controller command code\n\n");
      return false;
  }
}
bool parseMsgTracking(ControllerTracking& controller, char* buffer, int size) {
  if (size == 0) return false;
  char* parse_cur = buffer;
  char* parse_end = buffer + size;
  if (parse_cur[0] != 't') return false;
  ++parse_cur;
  char cmd = *parse_cur;
  ++parse_cur;

  std::pair<float, float> setpoint = controller.getSetpoint();
  float amt;
  switch (cmd) {
    case 'a':
      if (!parseFloat(&parse_cur, parse_end, ',', &setpoint.first))
        return false;
      if (!parseFloat(&parse_cur, parse_end, '\n', &setpoint.second))
        return false;
      break;
    case 'r':
      if (!parseFloat(&parse_cur, parse_end, '\n', &amt)) return false;
      setpoint.first += amt;
      break;
    case 'l':
      if (!parseFloat(&parse_cur, parse_end, '\n', &amt)) return false;
      setpoint.first -= amt;
      break;
    case 'u':
      if (!parseFloat(&parse_cur, parse_end, '\n', &amt)) return false;
      setpoint.second += amt;
      break;
    case 'd':
      if (!parseFloat(&parse_cur, parse_end, '\n', &amt)) return false;
      setpoint.second -= amt;
      break;
    default:
      return false;
  }
  controller.setSetpoint(setpoint);
  return true;
}

bool parseMsgSpray(Spray& spray, char* buffer, int size, Stream& serial) {
  if (size == 0) return false;
  char* parse_cur = buffer;
  char* parse_end = buffer + size;
  uint32_t cmd;
  if (parse_cur[0] != 's') return false;
  ++parse_cur;
  if (!parseInt(&parse_cur, parse_end, '\n', &cmd)) return false;

  switch (cmd) {
    case 0:
      serial.println("Spray off");
      spray.setSpray(false);
      return true;
    case 1:
      serial.println("Spray on");
      spray.setSpray(true);
      return true;
    default:
      serial.println("\n\nInvalid controller command code\n\n");
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
      if ((!human_serial::parseMsgRobot(robot_, odrive_, buffer, bufferi,
                                        serial_)) &&
          (!human_serial::parseMsgController(controller_, odrive_, buffer,
                                             bufferi, serial_)) &&
          (!human_serial::parseMsgSpray(spray_, buffer, bufferi, serial_)) &&
          (!human_serial::parseMsgCanPassthrough(odrive_, buffer, bufferi,
                                                 serial_)) &&
          (!custom_callback_(buffer, bufferi))) {
        serial_.println("Parse Error");
      };
      bufferi = 0;
    }
  }
}
