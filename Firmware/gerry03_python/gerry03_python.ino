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

#include <FlexCAN.h>

#include "can_simple.h"

#ifndef __MK66FX1M0__
  #error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif

struct CharBuffer_t;

// function declarations
uint8_t requestInfo(uint16_t node, uint8_t cmdId, bool rtr=false);
uint8_t sendBytes(uint16_t node, uint8_t cmdId, uint8_t *data, bool rtr=false);
static uint32_t parseOneInt(char delim);
static void read_serial();
static void read_serial(Stream &serial, CharBuffer_t &buf);
static bool checksum(CharBuffer_t &buf);
uint8_t sendInt32(uint16_t node, uint8_t cmdId, int32_t d1, int32_t d2 = 0);
uint8_t sendFloat(uint16_t node, uint8_t cmdId, float d1, float d2 = 0);

uint64_t last_hb_time[4];
static void read_packet(CAN_message_t inMsg) {
  uint8_t strbuf[13];
  strbuf[0] = inMsg.id >> 5;        // node
  strbuf[1] = inMsg.id & 0b11111;   // cmd
  memcpy(&strbuf[2], inMsg.buf, 8); // data
  strbuf[10] = -1;                  // end code
  strbuf[11] = 0;                   // end code
  strbuf[12] = -1;                  // end code
  Serial1.write(strbuf, 13);
  // ascii version
  if ((inMsg.id & 0b11111) == MSG_ODRIVE_HEARTBEAT) {
    if (millis() < (last_hb_time[inMsg.id >> 5] + 1000))
      return;
    else
      last_hb_time[inMsg.id >> 5] = millis();
  }
  // Serial.print(inMsg.id>>5);
  // Serial.print("\t");
  // Serial.print(inMsg.id & 0b11111, HEX);
  // Serial.print("\t - ");
  // uint8_t *bytePtr = inMsg.buf;
  // Serial.print(*(int32_t*)bytePtr);
  // Serial.print('\t');
  // Serial.print(*(int32_t*)(bytePtr+4));
  // Serial.print("\t - ");
  // Serial.print(*(float*)bytePtr);
  // Serial.print('\t');
  // Serial.println(*(float*)(bytePtr+4));
}

// -------------------------------------------------------------
void setup(void)
{
  delay(1000);
  Serial1.begin(115200);
  Serial.begin(115200);
  Serial1.println("Hello Teensy 3.6 dual CAN Test.");
  pinMode(13, OUTPUT);

  Can0.begin(500000);
  Can0.setNumTXBoxes(8);

  memset(last_hb_time, 0, sizeof last_hb_time);
}

// -------------------------------------------------------------
void loop(void)
{
  CAN_message_t inMsg;
  while (true) {
    while (Can0.available()) 
    {
      Can0.read(inMsg);
      read_packet(inMsg);
    }
    read_serial();
  }
}

struct CharBuffer_t
{
  char buffer[100];
  int bufferi = 0, parsei = 0;
};

CharBuffer_t buf, buf1;
static void read_serial() {
  read_serial(Serial, buf);
  read_serial(Serial1, buf1);
}
static void read_serial(Stream &serial, CharBuffer_t &buf) {
  while (serial.available()) {
    char c = serial.read();
    buf.buffer[buf.bufferi] = c;
    buf.bufferi++;
    if (c == '\n') {
      if (!checksum(buf)) {
        buf.bufferi = 0;
        buf.parsei = 0;
        continue;
      }
      uint8_t node = parseOneInt(buf, 'n');
      if (buf.bufferi == 0) continue; // this is how to tell if parse succeeded
      uint8_t cmd = parseOneInt(buf, 'c');
      if (buf.bufferi == 0) continue;

      switch (cmd) {
        case MSG_ODRIVE_ESTOP:
        case MSG_START_ANTICOGGING:
        case MSG_RESET_ODRIVE:
        case MSG_CLEAR_ERRORS:
          requestInfo(node, cmd);
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
          setCmd(node, cmd, buf);
          break;
        default:
          continue;
      }

      buf.bufferi = 0;
      buf.parsei = 0;
    }
  }
}

static bool checksum(CharBuffer_t &buf) {
  uint8_t sum = 0;
  for (int i = 0; i < (buf.bufferi-2); i++) {
    sum += buf.buffer[i];
  }
  return sum == buf.buffer[buf.bufferi-2];
}

static void setCmd(uint16_t node, uint8_t cmd, CharBuffer_t &buf) {
  switch (cmd) {
    case MSG_SET_AXIS_NODE_ID:
    case MSG_SET_AXIS_REQUESTED_STATE:
      sendInt32(node, cmd, parseOneInt(buf, '\n'));
      break;
    case MSG_SET_AXIS_STARTUP_CONFIG:
      Serial.println("Not yet implemented!");
      Serial1.println("Not yet implemented!");
      break;
    case MSG_SET_CONTROLLER_MODES:
      sendInt32(node, cmd, parseOneInt(buf, ','), parseOneInt(buf, '\n'));
      break;
    case MSG_SET_INPUT_POS:
      sendInputPos(node, cmd, parseOneFloat(buf, ','), parseOneInt(buf, ','), parseOneInt(buf, '\n'));
      break;
    case MSG_SET_INPUT_VEL:
    case MSG_SET_TRAJ_ACCEL_LIMITS:
      sendFloat(node, cmd, parseOneFloat(buf, ','), parseOneFloat(buf, '\n'));
      break;
    case MSG_SET_INPUT_TORQUE:
    case MSG_SET_VEL_LIMIT:
    case MSG_SET_TRAJ_VEL_LIMIT:
    case MSG_SET_TRAJ_INERTIA:
      sendFloat(node, cmd, parseOneFloat(buf, '\n'));
      break;
    default:
      Serial.println("Something went wrong");
      Serial1.println("Something went wrong");
  }
}

static uint32_t parseOneInt(CharBuffer_t &buf, char delim) {
  int starti = buf.parsei;
  while ((buf.parsei < buf.bufferi) && (buf.buffer[buf.parsei] != delim))
    buf.parsei++;
  if (buf.parsei == buf.bufferi) {
    Serial.println("Invalid parse!");
    Serial1.println("Invalid parse!");
    buf.bufferi = 0;
    buf.parsei = 0;
    return 0;
  }
  buf.buffer[buf.parsei] = 0; // requestInfonull termination
  buf.parsei++;
  return atoi(&buf.buffer[starti]);
}

static float parseOneFloat(CharBuffer_t &buf, char delim) {
  int starti = buf.parsei;
  while ((buf.parsei < buf.bufferi) && (buf.buffer[buf.parsei] != delim))
    buf.parsei++;
  if (buf.parsei == buf.bufferi) {
    Serial.println("Invalid parse!");
    Serial1.println("Invalid parse!");
    buf.bufferi = 0;
    return 0;
  }
  buf.buffer[buf.parsei] = 0; // null termination
  buf.parsei++;
  return atof(&buf.buffer[starti]);
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
  outMsg.rtr = rtr;
  outMsg.len = 8;
  outMsg.ext = 0;
  memcpy(outMsg.buf, data, 8);
  uint8_t success = Can0.write(outMsg);
  return success;
}
