#ifndef ODRIVE_CAN_H  // use old-style include guard for easier unit testing
#define ODRIVE_CAN_H

#include <array>
#include <FlexCAN_T4.h>

#include "../robot.h"
#include "../controllers/controller_interface.h"
#include "can_utils.h"
#include "can_simple.h"

namespace can_internal {
CanUtils<CAN1> Can0;
CanUtils<CAN2> Can1;
}  // namespace can_internal

/**
 * The Odrive object handles all communication with the odrive and should be the
 * only class that higher-level code interacts with.
 * TODO(gerry): namespace guard other files in `communication`
 * TODO(gerry): make a common interface, for unit testing and possible upgrades
 * to native UART (fibre)
 */
class Odrive {
 public:
  Odrive(Robot& robot, ControllerInterface& controller)
      : robot_(robot), controller_(controller) {}

  // Common API
  void setup();
  void update();

  //
  template <typename... Params>
  uint8_t send(uint8_t node, uint8_t cmd, Params&&... params);

 private:
  Robot& robot_;
  ControllerInterface& controller_;
  std::array<std::array<uint64_t, MSG_CLEAR_ERRORS + 1>, 4> last_received_us_;

  void parsePacket(const CAN_message_t& msg);
};

/******************************************************************************/

void Odrive::setup() {
  can_internal::Can0.begin();
  can_internal::Can0.setBaudRate(1000000);
  can_internal::Can1.begin();
  can_internal::Can1.setBaudRate(1000000);
  last_received_us_ = {};
}

void Odrive::update() {
  static CAN_message_t inMsg;
  while (can_internal::Can0.read(inMsg)) {
    parsePacket(inMsg);
  }
  while (can_internal::Can1.read(inMsg)) {
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
      return can_internal::Can0.send(node, cmd,
                                     std::forward<Params>(params)...);
    case 2:
    case 3:
      return can_internal::Can1.send(node, cmd,
                                     std::forward<Params>(params)...);
    default:
      // TODO: figure out what to do here...
      break;
  }
  return -1;
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
      robot_.winches.at(nodei).setError(
          can_internal::Can0.parse<uint32_t>(data));
      robot_.winches.at(nodei).setState(
          can_internal::Can0.parse<uint8_t>(data + 4));
      break;
    case MSG_GET_ENCODER_ESTIMATES: {
      static float pos, vel;
      can_internal::Can0.parseFloats(data, &pos, &vel);
      robot_.winches.at(nodei).setTheta(pos);
      robot_.winches.at(nodei).setThetaDot(vel);
      if (!controller_.encoderMsgCallback(this, nodei)) {
        send(nodei, 0b11111, true);  // service watchdog
      }
      break;
    }
    default:
      Serial.print("CAN bus 0:\tnode=");
      Serial.print(nodei);
      Serial.print("\tcmd=");
      Serial.print(cmd, HEX);
      Serial.print("\t\tdata = ");
      Serial.print(can_internal::Can0.parseInt32(data));
      Serial.print('\t');
      Serial.print(can_internal::Can0.parseInt32(data + 4));
      Serial.print('\t');
      Serial.print(can_internal::Can0.parseFloat(data));
      Serial.print('\t');
      Serial.println(can_internal::Can0.parseFloat((data + 4)));
      // TODO
      break;
  }
}

#endif
