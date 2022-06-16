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
#include "../spray.h"
#include "ascii_parser.h"

class Debug {
 public:
  Debug(Stream& serial, Robot& robot, ControllerInterface* controller,
        StateEstimatorInterface* estimator, Odrive& odrive, Spray& spray,
        bool (*custom_callback)(char* buffer, int size), int delay = 250)
      : serial_(serial),
        robot_(robot),
        controller_(controller),
        estimator_(estimator),
        odrive_(odrive),
        spray_(spray),
        custom_callback_(custom_callback),
        print_timer_(delay) {}

  // Common API
  void setup() {}
  void update() {
    if (print_timer_.check() && (serial_.availableForWrite() > 200)) {
      auto est_pos = estimator_->posEst();
      auto des_pos = controller_->setpointPos();
      serial_.printf("%7u - %d: %.4f %.4f %.4f - %.4f %.4f %.4f\t|\t",        //
                     micros() % 10000000,                                    //
                     controller_->getState(),                                //
                     estimator_->thetaEst(), est_pos.first, est_pos.second,  //
                     controller_->setpointTheta(), des_pos.first,
                     des_pos.second);
      for (int i = 0; i < 4; ++i) {
        const Winch& winch = robot_.winches.at(i);
        serial_.printf("%d %d %.4f %.4f\t|\t",  //
                       winch.error(), winch.state(),
                       print_raw_ ? winch.lenRaw() : winch.len(),
                       print_raw_ ? winch.lenDotRaw() : winch.lenDot());
      }
      serial_.println(spray_.spray());
    }
    if (serial_.availableForWrite() > 500) {
      controller_->writeSerial(serial_);
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
  Metro print_timer_;
  bool print_raw_ = false;

  void readSerial();
  bool parseMsgDebug(char* buffer, int size, Stream& serial);
};

namespace human_serial {
// TODO(gerry): move these functions into their respective classes

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

bool parseWinchZeroFromCurrentLength(Robot& robot, uint8_t winchi,
                                     char** parse_cur, char* parse_end,
                                     char delimiter, Stream& serial) {
  float cur_length;
  if (!parseFloat(parse_cur, parse_end, delimiter, &cur_length)) return false;
  serial.printf("Setting winch %d zero such that the current length is %.3f\n",
                winchi, cur_length);
  Winch& winch = robot.winches[winchi];
  winch.setZeroFromCurrentLength(cur_length);
  robot.saveZero(winchi);
  return true;
}

bool parseLengthCorrectionParams(const Robot& robot, uint8_t winchi,
                                 char** parse_cur, char* parse_end,
                                 char delimiter, Stream& serial) {
  float a, b, c;
  if (!parseFloat(parse_cur, parse_end, ',', &a)) return false;
  if (!parseFloat(parse_cur, parse_end, ',', &b)) return false;
  if (!parseFloat(parse_cur, parse_end, delimiter, &c)) return false;
  serial.printf(
      "Setting winch %d length correction parameters to {%.3f, %.3f, %.3f}\n",
      winchi, a, b, c);
  lenCorrectionParamsAll[winchi][0] = a;
  lenCorrectionParamsAll[winchi][1] = b;
  lenCorrectionParamsAll[winchi][2] = c;
  robot.saveLenCorrectionParams(winchi);
  return true;
}

bool parseMountPoints(const Robot& robot, uint8_t winchi, char** parse_cur,
                      char* parse_end, char delimiter, Stream& serial) {
  float x, y;
  if (!parseFloat(parse_cur, parse_end, ',', &x)) return false;
  if (!parseFloat(parse_cur, parse_end, delimiter, &y)) return false;
  serial.printf("Setting winch %d mount point to {%.3f, %.3f}\n", winchi, x, y);
  mountPoints[winchi][0] = x;
  mountPoints[winchi][1] = y;
  robot.saveMountPoint(winchi);
  return true;
}

bool parseMsgCalibration2(Robot& robot, char* buffer, int size,
                          Stream& serial) {
  if (size == 0) return false;
  char* parse_cur = buffer;
  char* parse_end = buffer + size;
  uint32_t cmd;
  if (parse_cur[0] != 'c') return false;
  ++parse_cur;
  if (!parseInt(&parse_cur, parse_end, ',', &cmd)) return false;

  switch (cmd) {
    case 30: // Set zero to current location
    case 31:
    case 32:
    case 33: {
      return parseWinchZeroFromCurrentLength(robot, cmd - 30, &parse_cur,
                                             parse_end, '\n', serial);
    }
    case 34: {
      for (int winchi = 0; winchi < 4; ++winchi) {
        if (!parseWinchZeroFromCurrentLength(robot, winchi, &parse_cur,
                                             parse_end, winchi < 3 ? ',' : '\n',
                                             serial))
          return false;
      }
      return true;
    }
    case 40:  // Set length correction parameters
    case 41:
    case 42:
    case 43: {
      return parseLengthCorrectionParams(robot, cmd - 40, &parse_cur, parse_end,
                                         '\n', serial);
    }
    case 44: {
      for (int winchi = 0; winchi < 4; ++winchi) {
        if (!parseLengthCorrectionParams(robot, winchi, &parse_cur, parse_end,
                                         winchi < 3 ? ',' : '\n', serial))
          return false;
      }
      return true;
    }
    case 50:  // Set mount points
    case 51:
    case 52:
    case 53: {
      return parseMountPoints(robot, cmd - 50, &parse_cur, parse_end, '\n',
                              serial);
    }
    case 54: {
      for (int winchi = 0; winchi < 4; ++winchi) {
        if (!parseMountPoints(robot, winchi, &parse_cur, parse_end,
                              winchi < 3 ? ',' : '\n', serial))
          return false;
      }
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
      if ((!parseMsgDebug(buffer, bufferi, serial_)) &&
          (!human_serial::parseMsgRobot(robot_, odrive_, buffer, bufferi,
                                        serial_)) &&
          (!human_serial::parseMsgCalibration2(robot_, buffer, bufferi,
                                               serial_)) &&
          (!human_serial::parseMsgController(controller_, odrive_, buffer,
                                             bufferi, serial_)) &&
          (!human_serial::parseMsgSpray(spray_, buffer, bufferi, serial_)) &&
          (!human_serial::parseMsgCanPassthrough(odrive_, buffer, bufferi,
                                                 serial_)) &&
          (!custom_callback_(buffer, bufferi)) &&
          (!controller_->readSerial(AsciiParser(buffer, bufferi), serial_))) {
        serial_.println("Parse Error");
      };
      bufferi = 0;
    }
  }
}

bool Debug::parseMsgDebug(char* buffer, int size, Stream& serial) {
  if (size == 0) return false;
  char* parse_cur = buffer;
  char* parse_end = buffer + size;
  uint32_t cmd;
  if (parse_cur[0] != 'd') return false;
  ++parse_cur;
  if (!human_serial::parseInt(&parse_cur, parse_end, '\n', &cmd)) return false;

  switch (cmd) {
    case 0:
    case 1:
      print_raw_ = cmd;
      return true;
    case 10: {  // print interval
      parse_cur = buffer + 1;
      if (!human_serial::parseInt(&parse_cur, parse_end, ',', &cmd))
        return false;
      if (cmd != 10) return false;
      int delay;
      if (!human_serial::parseInt(&parse_cur, parse_end, '\n', &delay))
        return false;
      print_timer_.interval(delay);
      return true;
    }
  }
  return false;
}
