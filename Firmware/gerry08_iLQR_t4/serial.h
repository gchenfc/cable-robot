#include "can_simple.h"

static uint32_t parseOneInt(char delim);
static float parseOneFloat(char delim);
static void setCmd(uint16_t node, uint8_t cmd);

char buffer[100];
int bufferi = 0, parsei = 0;
static void read_serial() {
  while (SerialD.available()) {
    char c = SerialD.read();
    buffer[bufferi] = c;
    bufferi++;
    if (c == '\n' || c == ';') {
      uint8_t node = parseOneInt('n');
      if (bufferi == 0) continue;
      uint8_t cmd = parseOneInt('c');
      if (bufferi == 0) continue;

      switch (cmd) {
        case 65 ... 75:
          // set selected motors to 0.2Nm torque
          if ((cmd - 60) & 0b1000) sendFloat(3, 0x0E, 0.2);
          if ((cmd - 60) & 0b0100) sendFloat(2, 0x0E, 0.2);
          if ((cmd - 60) & 0b0010) sendFloat(1, 0x0E, 0.2);
          if ((cmd - 60) & 0b0001) sendFloat(0, 0x0E, 0.2);
          break;
        case 80:
          setZero(node);
          break;
        case 50:
          start_closed_loop_1(node);
          break;
        case 51:
          stop_closed_loop_1(node);
          break;
        case 52:
          start_closed_loop_2();
          break;
        case 53:
          stop_closed_loop_2();
          break;
        case 54:
          start_closed_loop_4();
          break;
        case 55:
          stop_closed_loop_4();
          break;
        case 56:
          step_closed_loop_4_traj();
          break;
        case 57:
          start_closed_loop_4_traj();
          break;
        case 58:
          stop_closed_loop_4_traj();
          break;
        case 59:
          seti_closed_loop_4_traj(parseOneInt('\n'));
          break;
        case 60:
          resume_closed_loop_4_traj();
          break;
        case 61:
          pause_closed_loop_4_traj();
          break;
        case 100:
          manualpaint = parseOneInt('\n');
          break;
        case MSG_ODRIVE_ESTOP:
        case MSG_START_ANTICOGGING:
        case MSG_RESET_ODRIVE:
        case MSG_CLEAR_ERRORS:
          requestInfo(node, cmd);
          SerialD.print("Commanded ");
          SerialD.println(COMMANDS[cmd]);
          break;
        case MSG_GET_MOTOR_ERROR:
        case MSG_GET_ENCODER_ERROR:
        case MSG_GET_SENSORLESS_ERROR:
        case MSG_GET_ENCODER_ESTIMATES:
        case MSG_GET_ENCODER_COUNT:
        case MSG_GET_IQ:
        case MSG_GET_SENSORLESS_ESTIMATES:
        case MSG_GET_VBUS_VOLTAGE:
          requestInfo(node, cmd, true);
          SerialD.print("Requested ");
          SerialD.println(COMMANDS[cmd]);
          break;
        case MSG_SET_AXIS_NODE_ID:
        case MSG_SET_AXIS_REQUESTED_STATE:
        case MSG_SET_AXIS_STARTUP_CONFIG:
        case MSG_SET_CONTROLLER_MODES:
        case MSG_SET_INPUT_POS:
        case MSG_SET_INPUT_VEL:
        case MSG_SET_INPUT_TORQUE:
        case MSG_SET_VEL_LIMIT:
        case MSG_SET_TRAJ_VEL_LIMIT:
        case MSG_SET_TRAJ_ACCEL_LIMITS:
        case MSG_SET_TRAJ_INERTIA:
          setCmd(node, cmd);
          SerialD.print("Set ");
          SerialD.println(COMMANDS[cmd]);
          break;
        default:
          SerialD.println("Command not recognized!");
          SerialD.print(node);
          SerialD.print('\t');
          SerialD.println(cmd);
          continue;
      }

      bufferi = 0;
      parsei = 0;
    }
  }
}

static void setCmd(uint16_t node, uint8_t cmd) {
  switch (cmd) {
    case MSG_SET_AXIS_NODE_ID:
    case MSG_SET_AXIS_REQUESTED_STATE:
      sendInt32(node, cmd, parseOneInt('\n'));
      break;
    case MSG_SET_AXIS_STARTUP_CONFIG:
      Serial.println("Not yet implemented!");
      break;
    case MSG_SET_CONTROLLER_MODES:
      sendInt32(node, cmd, parseOneInt(','), parseOneInt('\n'));
      break;
    case MSG_SET_INPUT_POS:
      sendInputPos(node, cmd, parseOneFloat(','), parseOneInt(','), parseOneInt('\n'));
      break;
    case MSG_SET_INPUT_VEL:
    case MSG_SET_TRAJ_ACCEL_LIMITS:
      sendFloat(node, cmd, parseOneFloat(','), parseOneFloat('\n'));
      break;
    case MSG_SET_INPUT_TORQUE:
    case MSG_SET_VEL_LIMIT:
    case MSG_SET_TRAJ_VEL_LIMIT:
    case MSG_SET_TRAJ_INERTIA:
      sendFloat(node, cmd, parseOneFloat('\n'));
      break;
    default:
      SerialD.println("Something went wrong");
  }
}

bool isDelim(char c, char delim) {
  // manually add semicolon as a newline so that I can type multiple commands on
  // one line
  if (delim == '\n')
    return c == delim || c == ';';
  else
    return c == delim;
}

static uint32_t parseOneInt(char delim) {
  int starti = parsei;
  while ((parsei < bufferi) && (!isDelim(buffer[parsei], delim)))
    parsei++;
  if (parsei == bufferi) {
    SerialD.println("Invalid parse!");
    bufferi = 0;
    parsei = 0;
    return 0;
  }
  buffer[parsei] = 0; // requestInfonull termination
  parsei++;
  return atoi(&buffer[starti]);
}

static float parseOneFloat(char delim) {
  int starti = parsei;
  while ((parsei < bufferi) && (!isDelim(buffer[parsei], delim)))
    parsei++;
  if (parsei == bufferi) {
    SerialD.println("Invalid parse!");
    bufferi = 0;
    return 0;
  }
  buffer[parsei] = 0; // null termination
  parsei++;
  return atof(&buffer[starti]);
}
