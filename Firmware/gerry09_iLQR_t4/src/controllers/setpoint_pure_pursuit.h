#pragma once

#include "../communication/ascii_parser.h"
#include "../Vector.h"
#include "../spline.h"
#include "setpoint_basic.h"

/**
 * SetpointPurePursuit implements "pure pursuit" tracker that follows a single
 * waypoint (presumably input via serial).
 */
class SetpointPurePursuit : public SetpointBasic {
 public:
  using SetpointBasic::SetpointBasic;

  static void print_name(Stream& serialOut) {
    serialOut.print("SetpointPurePursuit");
  }

  /******************************** Common API ********************************/
  virtual void setup() override {
    SetpointBasic::setup();
    setpoint_ = defaultPos();
  }

  virtual bool readSerial(AsciiParser parser, Stream& serialOut) override;

  // We can just offload the pure pursuit logic to the travel stroke calculator
  virtual X desPos(float) const override { return setpoint_; }
  virtual V desVel(float) const override { return kZero<3>(); }
  virtual A desAcc(float) const override { return kZero<3>(); }
  virtual bool isDone(float) const override { return false; }
  virtual float timeTotal_s() const override { return 999999; }

 protected:
  X setpoint_;
};

bool SetpointPurePursuit::readSerial(AsciiParser parser, Stream& serialOut) {
  if (SetpointBasic::readSerial(parser, serialOut)) return true;

  UNWRAP_PARSE_CHECK(, parser.checkChar(SerialPrefixes::SETPOINT));
  UNWRAP_PARSE_CHECK(char cmd, parser.getChar(&cmd));

  switch (cmd) {
    case SetpointCommands::WAYPOINT_SET:
      UNWRAP_PARSE_CHECK(float x, parser.parseFloat(',', &x));
      UNWRAP_PARSE_CHECK(float y, parser.parseFloat(',', &y));
      UNWRAP_PARSE_CHECK(float theta, parser.parseFloat('\n', &theta));
      serialOut.printf("Waypoint setting waypoint to %.3f %.3f %.3f\n", x, y,
                       theta);
      updateSetpoint(X{x, y, theta});
      setpoint_ = X{x, y, theta};
      break;
    default:
      return false;
  }
  return true;
}
