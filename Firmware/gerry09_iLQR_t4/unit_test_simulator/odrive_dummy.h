#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

class Odrive {
 public:
  using Message = std::tuple<uint8_t, uint8_t, std::array<uint8_t, 8>, bool>;

  void setCallback(std::function<bool(Message)> cb) { cb_ = cb; }

  template <typename... Params>
  uint8_t send(uint8_t node, uint8_t cmd, Params &&...params) {
    return sendImpl(node, cmd, std::forward<Params>(params)...);
  }

 private:
  std::function<bool(Message)> cb_;

  uint8_t sendImpl(uint16_t node, uint8_t cmdId, int32_t d1, int32_t d2 = 0);
  uint8_t sendImpl(uint16_t node, uint8_t cmdId, float d1, float d2 = 0);
  uint8_t sendImpl(uint16_t node, uint8_t cmdId, float pos, int16_t vel,
                   int16_t torque);
  uint8_t sendImpl(uint16_t node, uint8_t cmdId, bool rtr = false);
  uint8_t sendImpl(uint16_t node, uint8_t cmdId, const uint8_t *data,
                   bool rtr = false);
};

uint8_t Odrive::sendImpl(uint16_t node, uint8_t cmdId, int32_t d1,
                          int32_t d2) {
  static uint8_t data[8];
  memcpy(&data, (uint8_t *)(&d1), 4);
  memcpy(((uint8_t *)&data) + 4, (uint8_t *)(&d2), 4);
  return sendImpl(node, cmdId, data);
}

uint8_t Odrive::sendImpl(uint16_t node, uint8_t cmdId, float d1, float d2) {
  static uint8_t data[8];
  if (isnan(d1) || isnan(d2)) {
    return 0;
  }
  memcpy(&data, (uint8_t *)(&d1), 4);
  memcpy(((uint8_t *)&data) + 4, (uint8_t *)(&d2), 4);
  return sendImpl(node, cmdId, data);
}

uint8_t Odrive::sendImpl(uint16_t node, uint8_t cmdId, float pos, int16_t vel,
                         int16_t torque) {
  static uint8_t data[8];
  memcpy(&data, (uint8_t *)(&pos), 4);
  memcpy(((uint8_t *)&data) + 4, (uint8_t *)(&vel), 2);
  memcpy(((uint8_t *)&data) + 6, (uint8_t *)(&torque), 2);
  return sendImpl(node, cmdId, data);
}

uint8_t Odrive::sendImpl(uint16_t node, uint8_t cmdId, bool rtr) {
  static const uint8_t can_data[8] = {0};
  return sendImpl(node, cmdId, can_data, rtr);
}

uint8_t Odrive::sendImpl(uint16_t node, uint8_t cmdId, const uint8_t *data,
                         bool rtr) {
  std::array<uint8_t, 8> data_arr;
  std::memcpy(data_arr.begin(), data, 8);
  return cb_(std::make_tuple(node, cmdId, data_arr, rtr));
}

#endif
