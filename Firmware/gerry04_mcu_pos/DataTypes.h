#pragma once

#include <stdint.h>
#include <FlexCAN.h>

struct Msg_t {
  uint8_t node;
  uint8_t cmd;
  uint8_t buf[8];

  Msg_t() {}
  Msg_t(uint8_t node, uint8_t cmd) : node(node), cmd(cmd) {}
  void fromCan(const CAN_message_t &inMsg) {
    node = inMsg.id >> 5;
    cmd = inMsg.id & 0b11111;
    memcpy(buf, inMsg.buf, 8);
  }
  void toCan(CAN_message_t &outMsg);
  void getInt32(int32_t &d1, int32_t &d2) {
    d1 = *(int32_t*) (&buf);
    d2 = *(int32_t*) (&buf+4);
  }
  void getUInt32(uint32_t &d1, uint32_t &d2) {
    d1 = *(uint32_t*) (&buf);
    d2 = *(uint32_t*) (&buf+4);
  }
  void getFloat(float &d1, float &d2) {
    d1 = *(float*) (&buf);
    d2 = *(float*) (&buf+4);
  }
  void getInputPos(float &pos, int16_t &vel, int16_t &torque) {
    pos = *(float*) (&buf);
    vel = *(int16_t*) (&buf+4);
    torque = *(int16_t*) (&buf+6);
  }
  void setInt32(int32_t d1, int32_t d2 = 0) {
    memcpy(&buf, (uint8_t *)(&d1), 4);
    memcpy(((uint8_t *)&buf) + 4, (uint8_t *)(&d2), 4);
  }
  void setFloat(float d1, float d2 = 0) {
    memcpy(&buf, (uint8_t *)(&d1), 4);
    memcpy(((uint8_t *)&buf) + 4, (uint8_t *)(&d2), 4);
  }
  void setInputPos(float pos, int16_t vel = 0, int16_t torque = 0) {
    memcpy(&buf, (uint8_t *)(&pos), 4);
    memcpy(((uint8_t *)&buf) + 4, (uint8_t *)(&vel), 2);
    memcpy(((uint8_t *)&buf) + 6, (uint8_t *)(&torque), 2);
  }
  
  void getInt32(int32_t &d1) {
    int32_t d2;
    getInt32(d1, d2);
  }
  void getFloat(float &d1) {
    float d2;
    getFloat(d1, d2);
  }
  void getUInt32(uint32_t &d1) {
    uint32_t d2;
    getUInt32(d1, d2);
  }
};

struct MsgBuffer_t {
  Msg_t buffer[100];
  uint size = 0;
  bool push(const Msg_t &inMsg) {
    if (size < 100) {
      memcpy(&buffer[size], &inMsg, sizeof(Msg_t));
      size++;
      return true;
    }
    return false;
  }
};

struct CharBuffer_t {
  char buffer[1000];
  uint size = 0;

  void append(uint8_t dat) {
    if (size > 999)
      clear();
    buffer[size] = dat;
    size++;
  }
  void clear() {
    size = 0;
  }
  bool isDone() const {
    return (size >= 3) &&
           (buffer[size-1] == 255) &&
           (buffer[size-2] == 0) &&
           (buffer[size-3] == 255);
  }
  uint8_t checksum(uint datasize) const {
    uint8_t sum = 0;
    for (int i = 0; i < datasize; i++)
      sum += buffer[i];
    return sum;
  }
  bool checksumIsGood() const {
    return (size == 14) && (checksum(size-4) == buffer[size-4]); 
  }
  void fillMsg(Msg_t &buf) const {
    buf.node = buffer[0];
    buf.cmd = buffer[1];
    memcpy(buf.buf, &buffer[2], 8);
  }
  void fillMsg(Msg_t *buf) const {
    buf->node = buffer[0];
    buf->cmd = buffer[1];
    memcpy(buf->buf, &buffer[2], 8);
  }
  void fillMe(const Msg_t &buf) {
    buffer[0] = buf.node;
    buffer[1] = buf.cmd;
    memcpy(&buffer[2], buf.buf, 8); // data
    buffer[10] = checksum(10);
    buffer[11] = -1;
    buffer[12] = 0;
    buffer[13] = -1;
    size = 14;
  }
};

struct CanBuffer_t {
  CAN_message_t buffer[100];
  int size = 0;
  bool push(const CAN_message_t &inMsg) {
    if (size < 100) {
      memcpy(&buffer[size], &inMsg, sizeof(CAN_message_t));
      size++;
      return true;
    }
    return false;
  }
};
