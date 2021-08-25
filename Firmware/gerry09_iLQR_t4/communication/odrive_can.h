#pragma once

#include <array>
#include <FlexCAN_T4.h>

#include "../robot.h"
#include "../controllers/controller_interface.h"
#include "can_utils.h"
#include "can_simple.h"

CanUtils<CAN1> Can0;
CanUtils<CAN2> Can1;

class Odrive {
 public:
  Odrive() {}

  // Common API
  void setup(Robot& robot);
  void update();

  //
  template <typename... Params>
  uint8_t send(uint8_t node, uint8_t cmd, Params&&... params);

  template <CanMsgCmdId ID>
  void addSubscriber(typename CanCmdTraits<ID>::RecvCallback);

 private:
  std::array<std::array<uint64_t, MSG_CLEAR_ERRORS + 1>, 4> last_received_us_;

  std::vector<std::function<void(Odrive*, CanUintPair)>> odrive_heartbeat_cb_;
  std::vector<std::function<void(Odrive*, uint64)>> get_motor_error_cb_;
  std::vector<std::function<void(Odrive*, uint32)>> get_encoder_error_cb_;
  std::vector<std::function<void(Odrive*, uint32)>> get_sensorless_error_cb_;
  std::vector<std::function<void(Odrive*, CanFloatPair)>>
      get_encoder_estimates_cb_;
  std::vector<std::function<void(Odrive*, CanIntPair)>> get_encoder_count_cb_;
  std::vector<std::function<void(Odrive*, CanFloatPair)>> get_iq_cb_;
  std::vector<std::function<void(Odrive*, CanFloatPair)>>
      get_sensorless_estimates_cb_;

  void parsePacket(const CAN_message_t& msg);
};

/******************************************************************************/

void Odrive::setup(Robot& robot) {
  Can0.begin();
  Can0.setBaudRate(1000000);
  Can1.begin();
  Can1.setBaudRate(1000000);
  last_received_us_ = {};

  robot.addSubscriber<MSG_ODRIVE_HEARTBEAT>([](Odrive* odrv, CanUIntPair data) {
    robot_.winches.at(nodei).setError(data.first);
    robot_.winches.at(nodei).setState(data.second);
  });
}

void Odrive::update() {
  static CAN_message_t inMsg;
  while (Can0.read(inMsg)) {
    parsePacket(inMsg);
  }
  while (Can1.read(inMsg)) {
    parsePacket(inMsg);
  }
}
template <typename... Params>
uint8_t Odrive::send(uint8_t node, uint8_t cmd, Params&&... params) {
  // TODO(gerry): find a way other than hard-coding to identify which node is on
  // which bus
  switch (node) {
    case 0:
    case 1:
      return Can0.send(node, cmd, std::forward<Params>(params)...);
    case 2:
    case 3:
      return Can1.send(node, cmd, std::forward<Params>(params)...);
    default:
      // TODO: figure out what to do here...
      break;
  }
  return -1;
}

#define ADD_SUBSCRIBER(TYPE, VARNAME)                                      \
  template <>                                                              \
  void addSubscriber<TYPE>(typename CanCmdTraits<TYPE>::RecvCallback cb) { \
    VARNAME.push_back(                                                     \
        std::bind(cb, std::placeholders::_1, std::placeholders::_2));      \
  }
ADD_SUBSCRIBER(MSG_ODRIVE_HEARTBEAT, odrive_heartbeat_cb_)
ADD_SUBSCRIBER(MSG_GET_MOTOR_ERROR, get_motor_error_cb_)
ADD_SUBSCRIBER(MSG_GET_ENCODER_ERROR, get_encoder_error_cb_)
ADD_SUBSCRIBER(MSG_GET_SENSORLESS_ERROR, get_sensorless_error_cb_)
ADD_SUBSCRIBER(MSG_GET_ENCODER_ESTIMATES, get_encoder_estimates_cb_)
ADD_SUBSCRIBER(MSG_GET_ENCODER_COUNT, get_encoder_count_cb_)
ADD_SUBSCRIBER(MSG_GET_IQ, get_iq_cb_)
ADD_SUBSCRIBER(MSG_GET_SENSORLESS_ESTIMATES, get_sensorless_estimates_cb_)
#undef ADD_SUBSCRIBER

void Odrive::parsePacket(const CAN_message_t& msg) {
  uint64_t time = micros();
  uint8_t nodei = msg.id >> 5;
  uint8_t cmd = msg.id & 0b11111;
  const uint8_t* data = msg.buf;

  // check for out-of-bound messages
  if (nodei > 4) return;
  if (cmd > MSG_CLEAR_ERRORS) return;

  last_received_us_.at(nodei).at(cmd) = time;
  switch (cmd) {
    case MSG_ODRIVE_HEARTBEAT:
      for (auto cb : odrive_heartbeat_cb_) cb(this, Can0.parse(data));
      break;
    case MSG_GET_ENCODER_ESTIMATES: {
      static float pos, vel;
      Can0.parseFloats(data, &pos, &vel);
      robot_.winches.at(nodei).setTheta(pos);
      robot_.winches.at(nodei).setThetaDot(vel);
      // send(nodei, MSG_SET_INPUT_TORQUE, controller_.get_torque_now(nodei));
      send(nodei, MSG_CO_HEARTBEAT_CMD, true);  // service watchdog
      break;
    }
      Serial.print("CAN bus 0:\tnode=");
      Serial.print(nodei);
      Serial.print("\tcmd=");
      Serial.print(cmd, HEX);
      Serial.print("\t\tdata = ");
      Serial.print(Can0.parseInt32(data));
      Serial.print('\t');
      Serial.print(Can0.parseInt32(data + 4));
      Serial.print('\t');
      Serial.print(Can0.parseFloat(data));
      Serial.print('\t');
      Serial.println(Can0.parseFloat((data + 4)));
      // TODO
      break;
  }
}
