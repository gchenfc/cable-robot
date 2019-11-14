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

#ifndef __MK66FX1M0__
  #error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif

static CAN_message_t msg;
static uint8_t hex[17] = "0123456789abcdef";

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
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
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
  Serial.print((int32_t)data.longs.long0);
  Serial.print('\t');
  Serial.print((int32_t)data.longs.long1);
  Serial.print('\t');
  Serial.print(data.floats.float0);
  Serial.print('\t');
  Serial.println(data.floats.float1);
}
static void msgDump2(uint8_t *bytePtr) {
  Serial.print(*(int32_t*)bytePtr);
  Serial.print('\t');
  Serial.print(*(int32_t*)(bytePtr+4));
  Serial.print('\t');
  Serial.print(*(float*)bytePtr);
  Serial.print('\t');
  Serial.println(*(float*)(bytePtr+4));
}

// -------------------------------------------------------------
void setup(void)
{
  delay(1000);
  Serial.begin(115200);
  Serial.println("Hello Teensy 3.6 dual CAN Test.");
  pinMode(13, OUTPUT);

  Can0.begin();  
  Can1.begin();

  //if using enable pins on a transceiver they need to be set on
  pinMode(2, OUTPUT);
  pinMode(35, OUTPUT);

  digitalWrite(2, HIGH);
  digitalWrite(35, HIGH);

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

  Can0.setNumTXBoxes(2);
}

int led = 0;
// -------------------------------------------------------------
void loop(void)
{
  CAN_message_t inMsg;
  while (true) {
    while (Can0.available()) 
    {
      Can0.read(inMsg);
      Serial.print("CAN bus 0:\tnode=");
      Serial.print(inMsg.id>>5);
      Serial.print("\tcmd=");
      Serial.print(inMsg.id & 0b11111, HEX);
      Serial.print("\t\tdata = ");
      msgDump2(inMsg.buf);
//      hexDump(8, inMsg.buf);
    }
    while (Can1.available())
    {
      Can1.read(inMsg);
      Serial.print("CAN bus 1 got a message: ");
      Serial.println(inMsg.id);
    }
    requestInfo(0, 0x00A);
    requestInfo(0, 0x017);
    delay(10);
  }
}

uint8_t requestInfo(uint16_t node, uint8_t cmdId) {
  CAN_message_t outMsg;
  outMsg.id = cmdId | (node << 5);
  outMsg.rtr = 1;
  msg.len = 8;
  msg.ext = 0;
  outMsg.buf[0] = 0;
  outMsg.buf[1] = 0;
  outMsg.buf[2] = 0;
  outMsg.buf[3] = 0;
  outMsg.buf[4] = 0;
  outMsg.buf[5] = 0;
  outMsg.buf[6] = 0;
  outMsg.buf[7] = 0;
  uint8_t success = Can0.write(outMsg);
  return success;
}
