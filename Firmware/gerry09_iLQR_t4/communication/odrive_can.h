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
  Odrive(Robot& robot, ControllerInterface& controller)
      : robot_(robot), controller_(controller) {}

  // Common API
  void setup();
  void update();

  //
  void parsePacket(const CAN_message_t& msg);

 private:
  Robot& robot_;
  ControllerInterface& controller_;
  std::array<std::array<uint64_t, MSG_CLEAR_ERRORS + 1>, 4> last_received_us_;
};

/******************************************************************************/

void Odrive::setup() {
  Can0.begin();
  Can0.setBaudRate(1000000);
  Can1.begin();
  Can1.setBaudRate(1000000);
  last_received_us_ = {};
}

void Odrive::update() {
  static CAN_message_t inMsg;
  while (Can0.read(inMsg)) {
    parsePacket(inMsg);
  }
}

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
      robot_.winches.at(nodei).setError(Can0.parse<uint32_t>(data));
      robot_.winches.at(nodei).setState(Can0.parse<uint32_t>(data + 4));
      break;
    case MSG_GET_ENCODER_ESTIMATES: {
      static float pos, vel;
      Can0.parseFloats(data, &pos, &vel);
      robot_.winches.at(nodei).setTheta(pos);
      robot_.winches.at(nodei).setThetaDot(vel);
      Can0.sendFloat(nodei, MSG_SET_INPUT_TORQUE,
                     controller_.get_torque_now(nodei));
      break;
    }
    default:
      // TODO
      break;
  }
}
