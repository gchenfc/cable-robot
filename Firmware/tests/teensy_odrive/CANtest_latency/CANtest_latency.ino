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
bool received[4] = {false, false, false, false};
uint64_t sentTime;
uint64_t totLatTime = 0;
uint16_t numMsgs = 0;
uint16_t numSucc = 0;

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
static void read_packet(CAN_message_t inMsg) {
  if ((inMsg.id & 0b11111) == MSG_GET_ENCODER_COUNT) {
    received[inMsg.id >> 5] = true;
  }
  // Serial.print("CAN bus 0:\tnode=");
  // Serial.print(inMsg.id>>5);
  // Serial.print("\tcmd=");
  // Serial.print(inMsg.id & 0b11111, HEX);
  // Serial.print("\t\tdata = ");
  // msgDump2(inMsg.buf);
}

void sendMsg() {
  numMsgs++;
  memset(received, false, 4);
  // received[2] = true;
  // received[3] = true;
  sentTime = micros();
  requestInfo(0, MSG_GET_ENCODER_COUNT, true);
  requestInfo(1, MSG_GET_ENCODER_COUNT, true);
  requestInfo(2, MSG_GET_ENCODER_COUNT, true);
  requestInfo(3, MSG_GET_ENCODER_COUNT, true);
}

// -------------------------------------------------------------
void setup(void)
{
  delay(1000);
  Serial.begin(115200);
  Serial.println("Hello Teensy 3.6 dual CAN Test.");
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

  sendMsg();
}

// -------------------------------------------------------------
CAN_message_t inMsg;
void loop(void)
{
    while (Can0.available()) 
    {
      Can0.read(inMsg);
      read_packet(inMsg);
    }
    
    if (micros() - sentTime > 10000) {
      Serial.println("Timed out... trying again");
      sendMsg();
    }
    // Serial.println("waiting...");
    if (!received[0] || !received[1] || !received[2] || !received[3])
      return;

    uint64_t receivedTime = micros();
    totLatTime += receivedTime - sentTime;
    numSucc++;
    Serial.print("round trip time: ");
    Serial.print(int(receivedTime - sentTime));
    Serial.print("us\t\tavg:");
    Serial.print(int(totLatTime / numSucc));
    Serial.print("us\t\tsuccess rate:");
    Serial.print(numSucc);
    Serial.print("/");
    Serial.print(numMsgs);
    Serial.println("");
    // delay(100);

    sendMsg();

    // if (micros() > (t + 1000)) {
    //   // requestInfo(0, MSG_GET_ENCODER_ESTIMATES);
    //   t = micros();
    //   memset(printAxes, true, 4);
    // }
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
