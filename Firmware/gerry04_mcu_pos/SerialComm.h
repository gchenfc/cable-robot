#pragma once

#include "DataTypes.h"

class OdriveComm;
class Controller;
class Axis;

class SerialComm {
 private:
  Stream &serial_;
  CharBuffer_t inBuf_;
  CharBuffer_t outBuf_;
  MsgBuffer_t msgBuf_;
  OdriveComm *odrv_;
  Axis *axis_;
  Controller *controller_;

 public:
  SerialComm(Stream &serial) : serial_(serial) {}

  void setOdrv(OdriveComm *odrv) { odrv_ = odrv; }
  void setController(Controller *controller) { controller_ = controller; }
  void setAxis(Axis axis[4]) { axis_ = axis; }

  void read();

  void write(const Msg_t &outMsg);
};
