/**
 * This is a unit test helper that interacts with a python dynamics simulator
 * Inputs/outputs:
 *  - "serial debug"
 *    - output is sent to the ostream passed into the constructor
 *    - inputs can be entered with "enterComputerInput(string)"
 *  - CAN
 *    - outputs are sent to the callback passed into the constructor
 *    - inputs can be entered with addMsgToQueue(can_message)
 */

#pragma once

#include <array>
#include <vector>

#include <arduino_test_utils.h>
#include "odrive_dummy.h"
#include "src/robot.h"
#include "src/state_estimators/state_estimator_first_order.h"
#include "src/controllers/controller_simple.h"
#include "src/communication/byte_packing.h"
#include "../src/communication/debug.h"

Odrive odrive;  // this is needed because controller expects a global extern
                // odrive :(
StringStreamer serial;

class CableRobotTester {
 public:
  using Message = std::tuple<uint8_t, uint8_t, std::array<uint8_t, 8>, bool>;
  using SendCallback = std::function<bool(Message)>;
  CableRobotTester(SendCallback callback, std::ostream& serial_out = std::cout)
      : robot_(),
        estimator_(new StateEstimatorFirstOrder(robot_)),
        controller_(new ControllerSimple(estimator_)),
        debug_(serial, robot_, controller_, estimator_, odrive),
        serial_out_(serial_out) {
    odrive.setCallback(callback);
  }

  void update() {
    for (auto msg : msgs_) {
      parsePacket(msg);
    }
    msgs_.clear();
    robot_.update();
    estimator_->update();
    debug_.update();
    controller_->update();
    serial_out_ << serial.getStream().str();
    serial.getStream().str("");
  }

  void addMsgToQueue(const Message& msg) { msgs_.push_back(msg); }

  void enterComputerInput(const std::string& s) { serial << s; }
  ControllerInterface* getController() { return controller_; }

 private:
  std::vector<Message> msgs_;
  Robot robot_;
  StateEstimatorInterface* estimator_;
  ControllerInterface* controller_;
  Debug debug_;

  std::ostream& serial_out_;

  void parsePacket(const Message& msg);
};

void CableRobotTester::parsePacket(const Message& msg) {
  const uint8_t& nodei = std::get<0>(msg);
  const uint8_t& cmd = std::get<1>(msg);
  uint8_t data[8];
  std::memcpy(data, std::get<2>(msg).begin(), 8);

  // check for out-of-bound messages
  if (nodei > 4) return;
  if (cmd > MSG_CLEAR_ERRORS) return;

  switch (cmd) {
    case MSG_ODRIVE_HEARTBEAT:
      robot_.winches.at(nodei).setError(read_le_unsafe<uint32_t>(data, 8));
      robot_.winches.at(nodei).setState(read_le_unsafe<uint32_t>(data + 4, 4));
      break;
    case MSG_GET_ENCODER_ESTIMATES: {
      static float pos, vel;
      read_le<float>(data, 8, pos);
      read_le<float>(data + 4, 4, vel);
      robot_.winches.at(nodei).setTheta(pos);
      robot_.winches.at(nodei).setThetaDot(vel);
      controller_->encoderMsgCallback(&odrive, nodei);
      break;
    }
    default:
      std::cout << "Message:\tnode="  //
                << nodei              //
                << "\tcmd="           //
                << cmd                //
                << std::endl;
      break;
  }
}
