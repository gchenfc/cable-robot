#include "OdriveComm.h"
#include "DataTypes.h"
#include "can_simple.h"

bool OdriveComm::isRtr(uint8_t cmd) {
  switch (cmd) {
    case MSG_GET_MOTOR_ERROR:
    case MSG_GET_ENCODER_ERROR:
    case MSG_GET_SENSORLESS_ERROR:
    case MSG_GET_ENCODER_ESTIMATES:
    case MSG_GET_ENCODER_COUNT:
    case MSG_GET_IQ:
    case MSG_GET_SENSORLESS_ESTIMATES:
    case MSG_GET_VBUS_VOLTAGE:
      return true;
  }
  return false;
}

bool OdriveComm::addToBuffer(const CAN_message_t &msg) {
  return canBuffer.push(msg);
}
bool OdriveComm::addToBuffer(const Msg_t &msg) {
  CAN_message_t outMsg;
  msg.toCan(outMsg);
  return canBuffer.push(outMsg);
}
bool OdriveComm::addToBuffer(const MsgBuffer_t &msgs) {
  CAN_message_t outMsg;
  for (int i = 0; i < msgs.size; i++) {
    msgs.buffer[i].toCan(outMsg);
    if (!canBuffer.push(outMsg))
      return false;
  }
  return true;
}

uint8_t OdriveComm::sendCanBuffer() {
  for (int i = canBuffer.size-1; i >= 0; i--) {
    uint8_t success = Can0.write(canBuffer.buffer[i]);
    if (!success) {
      canBuffer.size = i + 1;
      return 0;
    }
  }
  canBuffer.size = 0;
  return 1;
}

void OdriveComm::read() {
  while (can_.available()) {
    can_.read(inMsgCan);
    inMsg.fromCan(inMsgCan);
    if (inMsg.node < 4) {
      axis_[inMsg.node].readCanMsg(inMsg);
    }
    serial_.write(inMsg);
  }
}

void Axis::readCanMsg(Msg_t inMsg) {
  int32_t dum;
  switch (inMsg.cmd) {
    case MSG_CO_NMT_CTRL:
      break;
    case MSG_ODRIVE_HEARTBEAT:
      last_hb_time = millis();
      inMsg.getUInt32(error, state);
      break;
    case MSG_ODRIVE_ESTOP:
    case MSG_GET_MOTOR_ERROR:
      inMsg.getUInt32(motorError);
      break;
    case MSG_GET_ENCODER_ERROR:
      inMsg.getUInt32(encoderError);
      break;
    case MSG_GET_SENSORLESS_ERROR:
      inMsg.getUInt32(sensorlessError);
      break;
    case MSG_GET_ENCODER_ESTIMATES:
      inMsg.getFloat(posEst, velEst);
      break;
    case MSG_GET_ENCODER_COUNT:
      inMsg.getInt32(shadowCount, count_cpr);
      break;
    case MSG_GET_IQ:
      inMsg.getFloat(iqSet, iqMeas);
      break;
    case MSG_GET_SENSORLESS_ESTIMATES:
      inMsg.getFloat(posEstSensorless, velEstSensorless);
      break;
    case MSG_GET_VBUS_VOLTAGE:
      inMsg.getFloat(busVoltage);
      break;
  }
}

void Axis::readSerMsg(Msg_t inMsg, SerialComm &serial) {
  float dum;
  switch (inMsg.cmd) {
    case AXIS_CMDS_SET_ZERO:
      inMsg.getFloat(dum);
      setZeroPos(dum);
      break;
    case AXIS_CMDS_SET_ZERO_HERE:
      setZeroPos();
      break;
    case AXIS_CMDS_GET_ZERO:
      inMsg.setFloat(zeroPos);
      serial.write(inMsg);
      break;
    case AXIS_CMDS_SET_POS:
      inMsg.setFloat(dum);
      setPos(dum);
      break;
    case AXIS_CMDS_GET_POS:
      inMsg.setFloat(getPos());
      serial.write(inMsg);
      break;
  }
}
