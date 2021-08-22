#pragma once

#include <FlexCAN_T4.h>

#include "../robot.h"
#include "../controllers/controller_interface.h"
#include "can_utils.h"
#include "can_simple.h"

template <CAN_DEV_TABLE I0, CAN_DEV_TABLE I1>
class Odrive {
 public:
  using Can0 = FlexCAN_T4<I0, RX_SIZE_256, TX_SIZE_16>;
  using Can1 = FlexCAN_T4<I1, RX_SIZE_256, TX_SIZE_16>;
  Odrive(Can0& can0, Can1& can1, Robot& robot, ControllerInterface& controller)
      : can0_(can0), can1_(can1), robot_(robot), controller_(controller) {}

  // Common API
  void setup() {}
  void update() {}

 private:
  Can0& can0_;
  Can1& can1_;
  Robot& robot_;
  ControllerInterface& controller_;
};
