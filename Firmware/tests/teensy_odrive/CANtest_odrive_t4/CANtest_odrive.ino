// -------------------------------------------------------------
// CANtest for Teensy 3.6 dual CAN bus
// by Collin Kidder, Based on CANTest by Pawelsky (based on CANtest by teachop)
//
// Both buses are left at default 250k speed and the second bus sends frames to the first
// to do this properly you should have the two buses linked together. This sketch
// also assumes that you need to set enable pins active. Comment out if not using
// enable pins or set them to your correct pins.
//
// This sketch tests both buses as well as interrupt driven Rx and Tx. There are only
// two Tx buffers by default so sending 5 at a time forces the interrupt driven system
// to buffer the final three and send them via interrupts. All the while all Rx frames
// are internally saved to a software buffer by the interrupt handler.
//

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
#include "can_simple.h"

static CAN_message_t msg;
static uint8_t hex[17] = "0123456789abcdef";

// function declarations
uint8_t requestInfo(uint16_t node, uint8_t cmdId, bool rtr=false);
uint8_t sendBytes(uint16_t node, uint8_t cmdId, uint8_t *data, bool rtr=false);
static uint32_t parseOneInt(char delim);
static void read_serial();
uint8_t sendInt32(uint16_t node, uint8_t cmdId, int32_t d1, int32_t d2 = 0);
uint8_t sendFloat(uint16_t node, uint8_t cmdId, float d1, float d2 = 0);

static void msgDump2(uint8_t *bytePtr) {
  Serial.print(*(int32_t*)bytePtr);
  Serial.print('\t');
  Serial.print(*(int32_t*)(bytePtr+4));
  Serial.print('\t');
  Serial.print(*(float*)bytePtr);
  Serial.print('\t');
  Serial.println(*(float*)(bytePtr+4));
}
bool printAxes[4];
uint64_t return_times[4];
static void read_packet(CAN_message_t inMsg) {
  uint8_t node = inMsg.id >> 5;
  uint8_t cmd = inMsg.id & 0b11111;
  if (cmd == MSG_ODRIVE_HEARTBEAT) {
    // return;
    if (!printAxes[inMsg.id >> 5])
      return;
    else
      printAxes[inMsg.id >> 5] = false;
  }
  if (cmd == MSG_GET_ENCODER_ESTIMATES) {
    auto tmp = micros();
    auto period = tmp - return_times[node];
    return_times[node] = tmp;
    sendFloat(node, MSG_SET_INPUT_TORQUE, millis() / 100.0);
    requestInfo(node, MSG_GET_VBUS_VOLTAGE, true);
    Serial.print("\t\tTime since last update:  ");
    Serial.print(period);
    Serial.println(" us");
    return;
  }
  if (cmd == MSG_GET_VBUS_VOLTAGE) {
    auto tstop = micros();
    Serial.print("Got vbus voltage return!  ");
    Serial.print(tstop - return_times[node]);
    Serial.println(" us");
    return;
  }
  Serial.print("CAN bus 0:\tnode=");
  Serial.print(inMsg.id>>5);
  Serial.print("\tcmd=");
  Serial.print(inMsg.id & 0b11111, HEX);
  Serial.print("\t\tdata = ");
//      hexDump(8, inMsg.buf);");
  msgDump2(inMsg.buf);
}

// -------------------------------------------------------------
void setup(void)
{
  delay(1000);
  Serial.begin(115200);
  Serial.println("Hello Teensy 3.6 dual CAN Test.");
  pinMode(13, OUTPUT);

  Can0.begin();
  Can0.setBaudRate(1000000);

  msg.flags.extended = 0;
  msg.id = 0x500;
  msg.len = 4;
  msg.buf[0] = 10;
  msg.buf[1] = 20;
  msg.buf[2] = 0;
  msg.buf[3] = 100;
  msg.buf[4] = 128;
  msg.buf[5] = 64;
  msg.buf[6] = 32;
  msg.buf[7] = 16;
}

int led = 0;
// -------------------------------------------------------------
void loop(void)
{
  CAN_message_t inMsg;
  uint64_t t = millis();
  while (true) {
    while (Can0.read(inMsg))
    {
      read_packet(inMsg);
    }

    read_serial();

    if (millis() > (t + 1000)) {
      // requestInfo(0, MSG_GET_ENCODER_ESTIMATES);
      t = millis();
      memset(printAxes, true, 4);
      Serial.println("reset printAxes");
    }
    // requestInfo(0, 0x00A);
    // requestInfo(0, 0x017);
    // delay(10);
    // Serial.println("hi");
    // requestInfo(0, 0x002);
  }
}

char buffer[100];
int bufferi = 0, parsei = 0;
static void read_serial() {
  while (Serial.available()) {
    char c = Serial.read();
    buffer[bufferi] = c;
    bufferi++;
    if (c == '\n') {
      uint8_t node = parseOneInt('n');
      if (bufferi == 0) continue;
      uint8_t cmd = parseOneInt('c');
      if (bufferi == 0) continue;

      switch (cmd) {
        case MSG_ODRIVE_ESTOP:
        case MSG_START_ANTICOGGING:
        case MSG_RESET_ODRIVE:
        case MSG_CLEAR_ERRORS:
          requestInfo(node, cmd);
          Serial.print("Commanded ");
          Serial.println(COMMANDS[cmd]);
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
          Serial.print("Requested ");
          Serial.println(COMMANDS[cmd]);
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
          Serial.print("Set ");
          Serial.println(COMMANDS[cmd]);
          break;
        default:
          Serial.println("Command not recognized!");
          Serial.print(node);
          Serial.print('\t');
          Serial.println(cmd);
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
      Serial.println("Something went wrong");
  }
}

static uint32_t parseOneInt(char delim) {
  int starti = parsei;
  while ((parsei < bufferi) && (buffer[parsei] != delim))
    parsei++;
  if (parsei == bufferi) {
    Serial.println("Invalid parse!");
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
  while ((parsei < bufferi) && (buffer[parsei] != delim))
    parsei++;
  if (parsei == bufferi) {
    Serial.println("Invalid parse!");
    bufferi = 0;
    return 0;
  }
  buffer[parsei] = 0; // null termination
  parsei++;
  return atof(&buffer[starti]);
}

uint8_t requestInfo(uint16_t node, uint8_t cmdId, bool rtr) {
  uint8_t data[8];
  memset(data, 0, 8);
  return sendBytes(node, cmdId, data, rtr);
}

uint8_t sendInt32(uint16_t node, uint8_t cmdId, int32_t d1, int32_t d2) {
  uint8_t data[8];
  memcpy(&data, (uint8_t*) (&d1), 4);
  memcpy(((uint8_t *)&data)+4, (uint8_t*) (&d2), 4);
  return sendBytes(node, cmdId, data);
}

uint8_t sendFloat(uint16_t node, uint8_t cmdId, float d1, float d2) {
  uint8_t data[8];
  memcpy(&data, (uint8_t*) (&d1), 4);
  memcpy(((uint8_t *)&data)+4, (uint8_t*) (&d2), 4);
  return sendBytes(node, cmdId, data);
}

uint8_t sendInputPos(uint16_t node, uint8_t cmdId, float pos, int16_t vel, int16_t torque) {
  uint8_t data[8];
  memcpy(&data, (uint8_t*) (&pos), 4);
  memcpy(((uint8_t *)&data)+4, (uint8_t*) (&vel), 2);
  memcpy(((uint8_t *)&data)+6, (uint8_t*) (&torque), 2);
  return sendBytes(node, cmdId, data);
}

uint8_t sendBytes(uint16_t node, uint8_t cmdId, uint8_t *data, bool rtr) {
  CAN_message_t outMsg;
  outMsg.id = cmdId | (node << 5);
  outMsg.flags.remote = rtr;
  outMsg.len = 8;
  outMsg.flags.extended = 0;
  memcpy(outMsg.buf, data, 8);
  uint8_t success = Can0.write(outMsg);
  return success;
}
