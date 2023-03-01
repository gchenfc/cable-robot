#pragma once

#include "DataTypes.h"
#include "SerialComm.h"
#include "FlexCAN.h"
#include "can_simple.h"

class Axis;

class OdriveComm {
 protected:
  FlexCAN &can_;
  CAN_message_t inMsgCan;
  Msg_t inMsg;
  CanBuffer_t canBuffer;
  SerialComm &serial_;
  Axis *axis_;

 public:
  OdriveComm() {}
  OdriveComm(FlexCAN &can, SerialComm &serial, Axis axis[4]) : can_(can), serial_(serial), axis_(axis) {}

  static bool isRtr(uint8_t cmd);

  bool addToBuffer(const CAN_message_t &msg);
  bool addToBuffer(const Msg_t &msg);
  bool addToBuffer(const MsgBuffer_t &msgs);

  void read();

  uint8_t sendCanBuffer();
};

enum AxisCmds_t : uint8_t {
  AXIS_CMDS_SET_ZERO = 0,
  AXIS_CMDS_SET_ZERO_HERE = 1,
  AXIS_CMDS_GET_ZERO = 2,
  AXIS_CMDS_SET_POS = 3,
  AXIS_CMDS_GET_POS = 4,
};

class Axis {
 protected:
  uint8_t id_;
  uint64_t last_hb_time;
  uint32_t state;
  uint32_t error;
  uint32_t motorError;
  uint32_t encoderError;
  uint32_t sensorlessError;
  float posEst, velEst;
  int32_t shadowCount, count_cpr;
  int32_t controlMode, inputMode;
  float iqSet, iqMeas;
  float posEstSensorless, velEstSensorless;
  float busVoltage;
  float zeroPos;

  uint32_t believedControlMode = -1;

  OdriveComm *odrv_;
  Msg_t outCanMsg, outSerMsg;

 public:
  Axis() {}
  Axis(uint8_t id, OdriveComm *odrv) : id_(id), odrv_(odrv) {
    outCanMsg.node = id_;
    outSerMsg.node = id_+4;
  }

  void setId(uint8_t id) { id_ = id; }
  void setOdrv(OdriveComm *odrv) { odrv_ = odrv; }

  void readCanMsg(Msg_t inMsg);

  void readSerMsg(Msg_t inMsg, SerialComm &serial);

  void setZeroPos() {
    zeroPos = posEst;
  }
  void setZeroPos(float zeroPos) {
    this->zeroPos = zeroPos;
  }

  float getPos() const {
    return -(posEst - zeroPos);
  }

  void setPos(float pos, int16_t vel=0, int16_t torque=0) {
    if (believedControlMode != CTRL_MODE_POSITION_CONTROL) {
      outCanMsg.cmd = MSG_SET_CONTROLLER_MODES;
      outCanMsg.setInt32(CTRL_MODE_POSITION_CONTROL, 1);
      odrv_->addToBuffer(outCanMsg);
      believedControlMode = CTRL_MODE_POSITION_CONTROL;
    }
    outCanMsg.cmd = MSG_SET_INPUT_POS;
    outCanMsg.setInputPos(pos, vel, torque);
    odrv_->addToBuffer(outCanMsg);
  }

  void setVel(float vel, float torque=0) {
    if (believedControlMode != CTRL_MODE_VELOCITY_CONTROL) {
      outCanMsg.cmd = MSG_SET_CONTROLLER_MODES;
      outCanMsg.setInt32(CTRL_MODE_VELOCITY_CONTROL, 1);
      odrv_->addToBuffer(outCanMsg);
      believedControlMode = CTRL_MODE_VELOCITY_CONTROL;
    }
    outCanMsg.cmd = MSG_SET_INPUT_VEL;
    outCanMsg.setFloat(vel, torque);
    odrv_->addToBuffer(outCanMsg);
  }

  void setTorque(float torque) {
    if (believedControlMode != CTRL_MODE_CURRENT_CONTROL) {
      outCanMsg.cmd = MSG_SET_CONTROLLER_MODES;
      outCanMsg.setInt32(CTRL_MODE_CURRENT_CONTROL, 1);
      odrv_->addToBuffer(outCanMsg);
      believedControlMode = CTRL_MODE_CURRENT_CONTROL;
    }
    outCanMsg.cmd = MSG_SET_INPUT_TORQUE;
    outCanMsg.setFloat(torque);
    odrv_->addToBuffer(outCanMsg);
  }
  
  friend Controller;
};
