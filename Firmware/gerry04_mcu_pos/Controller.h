#pragma once

#include "DataTypes.h"
#include "OdriveComm.h"
#include <Metro.h>

enum ControllerCmds_t : uint8_t {
  CTRL_CMDS_SET_HEIGHTWIDTH = 5,
  CTRL_CMDS_GOTO_X = 6,
  CTRL_CMDS_GET_HEIGHTWIDTH,
  CTRL_CMDS_GET_X,
};

enum ControllerState_t {
  CTRL_STATE_IDLE,
  CTRL_STATE_GOING,
};

class Controller {
 protected:
  Axis *axis_;
  Msg_t outMsg_;

  float zeros[4] = {0,0,0,0};
  float height = 0, width = 0;
  float setpointx, setpointy;
  ControllerState_t state_ = CTRL_STATE_IDLE;

  Metro controllerTimer = Metro(100);

 public:
  Controller(Axis axis[4]) : axis_(axis) {
    outMsg_.node = 4;
  }

  void update();

  void processMsg(const Msg_t &msg, SerialComm &serial);

  void requesteState(ControllerState_t state);

  void cartesianPositionControl();

  void IK(float lengths[4], float x, float y) const;
  void FK(float &x, float &y, const float lengths[4]) const;
  void FK(float &x, float &y) const {
    float lengths[4];
    for (int i = 0; i < 4; i++) {
      lengths[i] = axis_[i].getPos();
    }
    FK(x, y, lengths);
  }
  void forceSolver(float tensions[4], float Fx, float Fy, float x, float y) const;
  void forceSolver(float tensions[4], float Fx, float Fy) {
    float x, y;
    FK(x, y);
    forceSolver(tensions, Fx, Fy, x, y);
  }
};
