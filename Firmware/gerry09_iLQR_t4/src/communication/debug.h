/**
 * @file debug.h
 * @author Gerry Chen
 * @brief This class is for printing/sending/saving debug information.
 */

#pragma once

#include <Metro.h>

#include "../controllers/controller_interface.h"
#include "../robot.h"
#include "../state_estimators/state_estimator_interface.h"
#include "odrive_can.h"
#include "../spray.h"
#include "ascii_parser.h"

bool default_callback(AsciiParser parser) { return false; }

class Debug {
 public:
  Debug(Stream& serial, Robot& robot, ControllerInterface* controller,
        StateEstimatorInterface* estimator, Odrive& odrive, Spray& spray,
        int delay = 250)
      : Debug(serial, robot, controller, estimator, odrive, spray,
              default_callback, delay) {}
  Debug(Stream& serial, Robot& robot, ControllerInterface* controller,
        StateEstimatorInterface* estimator, Odrive& odrive, Spray& spray,
        bool (*custom_callback)(AsciiParser), int delay = 250)
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
      serial_.printf("%7u - %d: %.4f %.4f %.4f - %.4f %.4f %.4f\t|\t",       //
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
  bool (*custom_callback_)(AsciiParser);
  Metro print_timer_;
  bool print_raw_ = false;

  void readSerial();
  bool parseMsgDebug(AsciiParser parser);
};

namespace human_serial {
// TODO(gerry): move these functions into their respective classes

bool parseWinchZeroFromCurrentLength(Robot& robot, uint8_t winchi,
                                     AsciiParser& parser, Stream& serial);
bool parseLengthCorrectionParams(const Robot& robot, uint8_t winchi,
                                 AsciiParser& parser, Stream& serial);
bool parseMountPoints(const Robot& robot, uint8_t winchi, AsciiParser& parser,
                      Stream& serial);

bool parseMsgCalibration(Robot& robot, Odrive& odrive, AsciiParser parser,
                         Stream& serial) {
  UNWRAP_PARSE_CHECK(, parser.checkChar('c'));
  UNWRAP_PARSE_CHECK(uint32_t cmd, parser.parseInt(&cmd));

  switch (cmd) {
    case 0:
    case 1:
    case 2:
    case 3: {
      UNWRAP_PARSE_CHECK(, parser.checkDone());
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
      UNWRAP_PARSE_CHECK(, parser.checkDone());
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
      UNWRAP_PARSE_CHECK(, parser.checkDone());
      uint8_t winchi = cmd - 10;
      serial.printf("Setting winch %d to zero and saving to EEPROM!\n", winchi);
      robot.setZero(winchi);
      robot.saveZero(winchi);
      serial.printf("Zeros:\nfloat kZeros[4] = {%.3f, %.3f, %.3f, %.3f};\n",
                    robot.zero(0), robot.zero(1), robot.zero(2), robot.zero(3));
      return true;
    }
    case 14: {
      UNWRAP_PARSE_CHECK(, parser.checkDone());
      serial.println("Setting all winches to zero and saving to EEPROM!");
      robot.setZeroAll();
      robot.saveZeros();
      serial.printf("Zeros:\nfloat kZeros[4] = {%.3f, %.3f, %.3f, %.3f};\n",
                    robot.zero(0), robot.zero(1), robot.zero(2), robot.zero(3));
      return true;
    }
    case 15: {
      UNWRAP_PARSE_CHECK(, parser.checkDone());
      serial.printf("Zeros:\nfloat kZeros[4] = {%.3f, %.3f, %.3f, %.3f};\n",
                    robot.zero(0), robot.zero(1), robot.zero(2), robot.zero(3));
      return true;
    }
    case 20:
    case 21:
    case 22:
    case 23: {
      UNWRAP_PARSE_CHECK(, parser.checkDone());
      uint8_t winchi = cmd - 20;
      serial.printf("Restoring winch %d zero from EEPROM!\n", winchi);
      robot.restoreZero(winchi);
      serial.printf("New zeros:\nfloat kZeros[4] = {%.3f, %.3f, %.3f, %.3f};\n",
                    robot.zero(0), robot.zero(1), robot.zero(2), robot.zero(3));
      return true;
    }
    case 24: {
      UNWRAP_PARSE_CHECK(, parser.checkDone());
      serial.println("Restoring all winch zeros from EEPROM!");
      robot.restoreZeros();
      serial.printf("New zeros:\nfloat kZeros[4] = {%.3f, %.3f, %.3f, %.3f};\n",
                    robot.zero(0), robot.zero(1), robot.zero(2), robot.zero(3));
      return true;
    }
    case 30:  // Set zero to current location
    case 31:
    case 32:
    case 33: {
      UNWRAP_PARSE_CHECK(float a, parser.peek(&a));
      return parseWinchZeroFromCurrentLength(robot, cmd - 30, parser, serial) &&
             parser.checkDone();
    }
    case 34: {
      UNWRAP_PARSE_CHECK(float a, parser.peek(&a, &a, &a, &a));
      for (int winchi = 0; winchi < 4; ++winchi) {
        if (!parseWinchZeroFromCurrentLength(robot, winchi, parser, serial))
          return false;
      }
      return parser.checkDone();
    }
    case 40:  // Set length correction parameters
    case 41:
    case 42:
    case 43: {
      UNWRAP_PARSE_CHECK(float a, parser.peek(&a, &a, &a));
      return parseLengthCorrectionParams(robot, cmd - 40, parser, serial);
    }
    case 44: {
      UNWRAP_PARSE_CHECK(
          float a, parser.peek(&a, &a, &a, &a, &a, &a, &a, &a, &a, &a, &a, &a));
      for (int winchi = 0; winchi < 4; ++winchi) {
        if (!parseLengthCorrectionParams(robot, winchi, parser, serial))
          return false;
      }
      return true;
    }
    case 50:  // Set mount points
    case 51:
    case 52:
    case 53: {
      UNWRAP_PARSE_CHECK(float a, parser.peek(&a, &a));
      return parseMountPoints(robot, cmd - 50, parser, serial);
    }
    case 54: {
      UNWRAP_PARSE_CHECK(float a, parser.peek(&a, &a, &a, &a, &a, &a, &a, &a));
      for (int winchi = 0; winchi < 4; ++winchi) {
        if (!parseMountPoints(robot, winchi, parser, serial)) return false;
      }
      return true;
    }
  }
  return false;
}

bool parseWinchZeroFromCurrentLength(Robot& robot, uint8_t winchi,
                                     AsciiParser& parser, Stream& serial) {
  UNWRAP_PARSE_CHECK(float cur_length, parser.parseFloat(&cur_length));
  serial.printf("Setting winch %d zero such that the current length is %.3f\n",
                winchi, cur_length);
  Winch& winch = robot.winches[winchi];
  winch.setZeroFromCurrentLength(cur_length);
  robot.saveZero(winchi);
  return true;
}

bool parseLengthCorrectionParams(const Robot& robot, uint8_t winchi,
                                 AsciiParser& parser, Stream& serial) {
  UNWRAP_PARSE_CHECK(float a, parser.parseFloat(&a));
  UNWRAP_PARSE_CHECK(float b, parser.parseFloat(&b));
  UNWRAP_PARSE_CHECK(float c, parser.parseFloat(&c));
  serial.printf(
      "Setting winch %d length correction parameters to {%.3f, %.3f, %.3f}\n",
      winchi, a, b, c);
  lenCorrectionParamsAll[winchi][0] = a;
  lenCorrectionParamsAll[winchi][1] = b;
  lenCorrectionParamsAll[winchi][2] = c;
  robot.saveLenCorrectionParams(winchi);
  return true;
}

bool parseMountPoints(const Robot& robot, uint8_t winchi, AsciiParser& parser,
                      Stream& serial) {
  UNWRAP_PARSE_CHECK(float x, parser.parseFloat(&x));
  UNWRAP_PARSE_CHECK(float y, parser.parseFloat(&y));
  serial.printf("Setting winch %d mount point to {%.3f, %.3f}\n", winchi, x, y);
  mountPoints[winchi][0] = x;
  mountPoints[winchi][1] = y;
  robot.saveMountPoint(winchi);
  return true;
}

bool parseMsgController(ControllerInterface* controller, Odrive& odrive,
                        AsciiParser parser, Stream& serial) {
  UNWRAP_PARSE_CHECK(, parser.checkChar('g'));
  UNWRAP_PARSE_CHECK(uint32_t cmd, parser.parseInt(&cmd));
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

bool parseMsgSpray(Spray& spray, AsciiParser parser, Stream& serial) {
  UNWRAP_PARSE_CHECK(, parser.checkChar('s'));
  // First check for 0 and 1, for backwards compatibility.
  AsciiParser parser0 = parser, parser1 = parser;
  if (parser0.checkChar('0') && parser0.checkChar('\n')) {
    serial.println("Spray off");
    spray.setSpray(false);
    return true;
  } else if (parser1.checkChar('1') && parser1.checkChar('\n')) {
    serial.println("Spray on");
    spray.setSpray(true);
    return true;
  } else {
    // Forward rest of args to spray paint MCU
    spray.forward_msg(parser.get_buffer_cur(), parser.len());
    return true;
  }
}

bool parseMsgCanPassthrough(Odrive& odrive, AsciiParser parser,
                            Stream& serial) {
  UNWRAP_PARSE_CHECK(uint8_t node, parser.parseInt('n', &node))
  UNWRAP_PARSE_CHECK(uint8_t cmd, parser.parseInt('c', &cmd)) // THIS WAS A TYPO AND MIGHT BE 'n'

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
      UNWRAP_PARSE_CHECK(, parser.parse(&i1));
      odrive.send(node, cmd, i1);
      break;
    case MSG_SET_AXIS_STARTUP_CONFIG:
      serial.println("Not yet implemented!");
      break;
    case MSG_SET_CONTROLLER_MODES:
      UNWRAP_PARSE_CHECK(, parser.parse(&i1, &i2));
      odrive.send(node, cmd, i1, i2);
      break;
    case MSG_SET_INPUT_POS:
      UNWRAP_PARSE_CHECK(, parser.parse(&f1, &i1, &i2));
      odrive.send(node, cmd, f1, i1, i2);
      break;
    case MSG_SET_INPUT_VEL:
    case MSG_SET_TRAJ_ACCEL_LIMITS:
      UNWRAP_PARSE_CHECK(, parser.parse(&f1, &f2));
      odrive.send(node, cmd, f1, f2);
      break;
    case MSG_SET_INPUT_TORQUE:
    case MSG_SET_VEL_LIMIT:
    case MSG_SET_TRAJ_VEL_LIMIT:
    case MSG_SET_TRAJ_INERTIA:
      UNWRAP_PARSE_CHECK(, parser.parse(&f1));
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
      AsciiParser parser(buffer, bufferi);
      if ((!parseMsgDebug(parser)) &&
          (!human_serial::parseMsgCalibration(robot_, odrive_, parser,
                                              serial_)) &&
          (!human_serial::parseMsgController(controller_, odrive_, parser,
                                             serial_)) &&
          (!human_serial::parseMsgSpray(spray_, parser, serial_)) &&
          (!human_serial::parseMsgCanPassthrough(odrive_, parser, serial_)) &&
          (!custom_callback_(parser)) &&
          (!controller_->readSerial(AsciiParser(buffer, bufferi), serial_))) {
        serial_.println("Parse Error");
      };
      bufferi = 0;
    }
  }
}

bool Debug::parseMsgDebug(AsciiParser parser) {
  UNWRAP_PARSE_CHECK(, parser.checkChar('d'));
  UNWRAP_PARSE_CHECK(uint32_t cmd, parser.parseInt(&cmd));

  switch (cmd) {
    case 0:
    case 1:
      print_raw_ = cmd;
      return parser.checkDone();
    case 10: {  // print interval
      UNWRAP_PARSE_CHECK(int delay, parser.parseInt(&delay));
      if (parser.checkDone()) {
        print_timer_.interval(delay);
        return true;
      }
    }
  }

  return false;
}
