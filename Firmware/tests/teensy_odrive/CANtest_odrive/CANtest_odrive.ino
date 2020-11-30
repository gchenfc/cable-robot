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

static CAN_message_t msg;
static uint8_t hex[17] = "0123456789abcdef";

// function declarations
uint8_t requestInfo(uint16_t node, uint8_t cmdId, bool rtr=false);
uint8_t sendBytes(uint16_t node, uint8_t cmdId, uint8_t *data, bool rtr=false);
static uint32_t parseOneInt(char delim);
static void read_serial();
uint8_t sendInt32(uint16_t node, uint8_t cmdId, int32_t d1, int32_t d2 = 0);
uint8_t sendFloat(uint16_t node, uint8_t cmdId, float d1, float d2 = 0);



union DataConversion_t
{
  struct {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
  } bytes;
  struct {
    uint32_t long0;
    uint32_t long1;
  } longs;
  struct {
    float float0;
    float float1;
  } floats;
};
// -------------------------------------------------------------
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial1.write( hex[ working>>4 ] );
    Serial1.write( hex[ working&15 ] );
  }
  Serial1.write('\r');
  Serial1.write('\n');
}
static void msgDump(uint8_t *bytePtr) {
  DataConversion_t data;
  data.bytes.byte0 = bytePtr[0];
  data.bytes.byte1 = bytePtr[1];
  data.bytes.byte2 = bytePtr[2];
  data.bytes.byte3 = bytePtr[3];
  data.bytes.byte4 = bytePtr[4];
  data.bytes.byte5 = bytePtr[5];
  data.bytes.byte6 = bytePtr[6];
  data.bytes.byte7 = bytePtr[7];
  Serial1.print((int32_t)data.longs.long0);
  Serial1.print('\t');
  Serial1.print((int32_t)data.longs.long1);
  Serial1.print('\t');
  Serial1.print(data.floats.float0);
  Serial1.print('\t');
  Serial1.println(data.floats.float1);
}
static void msgDump2(uint8_t *bytePtr) {
  Serial1.print(*(int32_t*)bytePtr);
  Serial1.print('\t');
  Serial1.print(*(int32_t*)(bytePtr+4));
  Serial1.print('\t');
  Serial1.print(*(float*)bytePtr);
  Serial1.print('\t');
  Serial1.println(*(float*)(bytePtr+4));
}
bool printAxes[4];
static void read_packet(CAN_message_t inMsg) {
  if ((inMsg.id & 0b11111) == MSG_ODRIVE_HEARTBEAT) {
    // return;
    if (!printAxes[inMsg.id >> 5])
      return;
    else
      printAxes[inMsg.id >> 5] = false;
  }
  Serial1.print("CAN bus 0:\tnode=");
  Serial1.print(inMsg.id>>5);
  Serial1.print("\tcmd=");
  Serial1.print(inMsg.id & 0b11111, HEX);
  Serial1.print("\t\tdata = ");
//      hexDump(8, inMsg.buf);");
  msgDump2(inMsg.buf);
}

// -------------------------------------------------------------
void setup(void)
{
  delay(1000);
  Serial1.begin(115200);
  Serial1.println("Hello Teensy 3.6 dual CAN Test.");
  pinMode(13, OUTPUT);

  Can0.begin(500000);

  msg.ext = 0;
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

  Can0.setNumTXBoxes(8);
}

int led = 0;
// -------------------------------------------------------------
void loop(void)
{
  CAN_message_t inMsg;
  uint64_t t = millis();
  while (true) {
    while (Can0.available()) 
    {
      Can0.read(inMsg);
      read_packet(inMsg);
    }

    read_serial();

    if (millis() > (t + 1000)) {
      // requestInfo(0, MSG_GET_ENCODER_ESTIMATES);
      t = millis();
      memset(printAxes, true, 4);
    }
    // requestInfo(0, 0x00A);
    // requestInfo(0, 0x017);
    delay(10);
    // Serial1.println("hi");
    // requestInfo(0, 0x002);
  }
}

char buffer[100];
int bufferi = 0, parsei = 0;
static void read_serial() {
  while (Serial1.available()) {
    char c = Serial1.read();
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
          Serial1.print("Commanded ");
          Serial1.println(COMMANDS[cmd]);
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
          Serial1.print("Requested ");
          Serial1.println(COMMANDS[cmd]);
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
          Serial1.print("Set ");
          Serial1.println(COMMANDS[cmd]);
          break;
        default:
          Serial1.println("Command not recognized!");
          Serial1.print(node);
          Serial1.print('\t');
          Serial1.println(cmd);
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
      Serial1.println("Something went wrong");
  }
}

static uint32_t parseOneInt(char delim) {
  int starti = parsei;
  while ((parsei < bufferi) && (buffer[parsei] != delim))
    parsei++;
  if (parsei == bufferi) {
    Serial1.println("Invalid parse!");
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
    Serial1.println("Invalid parse!");
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
  outMsg.rtr = rtr;
  outMsg.len = 8;
  outMsg.ext = 0;
  memcpy(outMsg.buf, data, 8);
  uint8_t success = Can0.write(outMsg);
  return success;
}
