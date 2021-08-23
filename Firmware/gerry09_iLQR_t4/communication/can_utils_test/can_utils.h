#pragma once

#include <Stream.h>
#include <FlexCAN_T4.h>

template <CAN_DEV_TABLE _bus>
class CanUtils : public FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16> {
 public:
  using Base = FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16>;

  uint8_t requestInfo(uint16_t node, uint8_t cmdId, bool rtr = false);

  // templated parses
  template <typename T>
  static void parse(const uint8_t *data, T *out);
  template <typename T>
  static T parse(const uint8_t *data);
  // convenience aliases
  static void parseFloat(const uint8_t *data, float *f) { parse(data, f); }
  static void parseInt32(const uint8_t *data, int32_t *i) { parse(data, i); }
  static float parseFloat(const uint8_t *data) { return parse<float>(data); }
  static int32_t parseInt32(const uint8_t *data) {
    return parse<int32_t>(data);
  }
  // parse two at a time
  static void parseFloats(const uint8_t *data, float *float1, float *float2);
  static void parseInt32s(const uint8_t *data, int32_t *i1, int32_t *i2);
  static std::pair<float, float> parseFloats(const uint8_t *data) {
    return parse<std::pair<float, float>>(data);
  }
  static std::pair<int32_t, int32_t> parseInt32s(const uint8_t *data) {
    return parse<std::pair<int32_t, int32_t>>(data);
  }

  uint8_t sendInt32(uint16_t node, uint8_t cmdId, int32_t d1, int32_t d2 = 0);
  uint8_t sendFloat(uint16_t node, uint8_t cmdId, float d1, float d2 = 0,
                    Stream &debug_serial = Serial);
  uint8_t sendInput_pos(uint16_t node, uint8_t cmdId, float pos, int16_t vel,
                        int16_t torque);
  uint8_t sendBytes(uint16_t node, uint8_t cmdId, const uint8_t *data,
                    bool rtr = false);
};

/////////////////////

template <CAN_DEV_TABLE T>
uint8_t CanUtils<T>::requestInfo(uint16_t node, uint8_t cmdId, bool rtr) {
  static const uint8_t can_data[8] = {0};
  return sendBytes(node, cmdId, can_data, rtr);
}

template <CAN_DEV_TABLE T1>
template <typename T2>
void CanUtils<T1>::parse(const uint8_t *data, T2 *f) {
  static_assert(sizeof(T2) <= 8, "Data type too large for 8-byte CAN data");
  memcpy(f, data, sizeof(T2));
}
template <CAN_DEV_TABLE T1>
template <typename T2>
T2 CanUtils<T1>::parse(const uint8_t *data) {
  static T2 tmp;
  memcpy(&tmp, data, sizeof(T2));
  return tmp;
}

template <CAN_DEV_TABLE T>
void CanUtils<T>::parseFloats(const uint8_t *data, float *float1,
                              float *float2) {
  memcpy(float1, data, 4);
  memcpy(float2, data + 4, 4);
}
template <CAN_DEV_TABLE T>
void CanUtils<T>::parseInt32s(const uint8_t *data, int32_t *i1, int32_t *i2) {
  memcpy(i1, data, 4);
  memcpy(i2, data + 4, 4);
}

template <CAN_DEV_TABLE T>
uint8_t CanUtils<T>::sendInt32(uint16_t node, uint8_t cmdId, int32_t d1,
                               int32_t d2) {
  static uint8_t data[8];
  memcpy(&data, (uint8_t *)(&d1), 4);
  memcpy(((uint8_t *)&data) + 4, (uint8_t *)(&d2), 4);
  return sendBytes(node, cmdId, data);
}

template <CAN_DEV_TABLE T>
uint8_t CanUtils<T>::sendFloat(uint16_t node, uint8_t cmdId, float d1, float d2,
                               Stream &debug_serial) {
  static uint8_t data[8];
  if (isnan(d1) || isnan(d2)) {
    debug_serial.print("Tried to send nan: ");
    debug_serial.print(node);
    debug_serial.print(" - ");
    debug_serial.println(cmdId);
    return 0;
  }
  memcpy(&data, (uint8_t *)(&d1), 4);
  memcpy(((uint8_t *)&data) + 4, (uint8_t *)(&d2), 4);
  return sendBytes(node, cmdId, data);
}

template <CAN_DEV_TABLE T>
uint8_t CanUtils<T>::sendInput_pos(uint16_t node, uint8_t cmdId, float pos,
                                   int16_t vel, int16_t torque) {
  static uint8_t data[8];
  memcpy(&data, (uint8_t *)(&pos), 4);
  memcpy(((uint8_t *)&data) + 4, (uint8_t *)(&vel), 2);
  memcpy(((uint8_t *)&data) + 6, (uint8_t *)(&torque), 2);
  return sendBytes(node, cmdId, data);
}

template <CAN_DEV_TABLE T>
uint8_t CanUtils<T>::sendBytes(uint16_t node, uint8_t cmdId,
                               const uint8_t *data, bool rtr) {
  static CAN_message_t outMsg;
  outMsg.id = cmdId | (node << 5);
  outMsg.flags.remote = rtr;
  outMsg.len = 8;
  outMsg.flags.extended = 0;
  memcpy(outMsg.buf, data, 8);
  uint8_t success = Base::write(outMsg);
  return success;
}
