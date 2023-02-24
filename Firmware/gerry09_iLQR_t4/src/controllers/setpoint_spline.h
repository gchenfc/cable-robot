#pragma once

#include "../communication/ascii_parser.h"
#include "../Vector.h"
#include "../spline.h"
#include "setpoint_basic.h"

/**
 * SetpointSpline implements a spline tracking setpoint calculator.
 */
class SetpointSpline : public SetpointBasic {
 public:
  using SetpointBasic::SetpointBasic;

  static void print_name(Stream& serialOut) {
    serialOut.print("SetpointSpline");
  }

  /******************************** Common API ********************************/
  virtual void setup() override {
    SetpointBasic::setup();
    spline_.reset();
  }

  virtual bool readSerial(AsciiParser parser, Stream& serialOut) override;

  virtual X desPos(float t) override {
    return appendZeroTheta(spline_.eval(t));
  }
  virtual V desVel(float t) override {
    return appendZeroTheta(spline_.evald(t));
  }
  virtual A desAcc(float t) override {
    return appendZeroTheta(spline_.evaldd(t));
  }
  virtual bool isDone(float t) override { return (t >= spline_.duration()); }

 protected:
  PPoly<750> spline_;
};

bool SetpointSpline::readSerial(AsciiParser parser, Stream& serialOut) {
  if (SetpointBasic::readSerial(parser, serialOut)) return true;

  UNWRAP_PARSE_CHECK(, parser.checkChar(SerialPrefixes::SETPOINT));
  UNWRAP_PARSE_CHECK(char cmd, parser.getChar(&cmd));

  switch (cmd) {
    case SetpointCommands::SPLINE_RESET:
      serialOut.println("Spline reset.");
      spline_.reset();
      break;
    case SetpointCommands::SPLINE_ADD_SEGMENT:
      UNWRAP_PARSE_CHECK(float t, parser.parseFloat(',', &t));
      UNWRAP_PARSE_CHECK(float cx3, parser.parseFloat(',', &cx3));
      UNWRAP_PARSE_CHECK(float cx2, parser.parseFloat(',', &cx2));
      UNWRAP_PARSE_CHECK(float cx1, parser.parseFloat(',', &cx1));
      UNWRAP_PARSE_CHECK(float cx0, parser.parseFloat(',', &cx0));
      UNWRAP_PARSE_CHECK(float cy3, parser.parseFloat(',', &cy3));
      UNWRAP_PARSE_CHECK(float cy2, parser.parseFloat(',', &cy2));
      UNWRAP_PARSE_CHECK(float cy1, parser.parseFloat(',', &cy1));
      UNWRAP_PARSE_CHECK(float cy0, parser.parseFloat('\n', &cy0));

      if (!spline_.add_segment(
              t, {{{{cx3, cx2, cx1, cx0}}, {{cy3, cy2, cy1, cy0}}}})) {
        serialOut.println(
            "Spline Error adding next segment of spline: out of space!");
      }
      break;
    case SetpointCommands::POLL_SPLINE_STATUS:
      serialOut.printf(
          "Spline Status: %d %.3f %.3f\n", state_,
          (state_ == State::RUNNING) ? time_s() : time_s(t_paused_us_),
          spline_.duration());
      break;
    case SetpointCommands::SPLINE_DEBUG_PRINT_NUM_SEGMENTS:
      serialOut.printf("Spline number of segments: %d\n",
                       spline_.get_n_segments());
      break;
    case SetpointCommands::SPLINE_DEBUG_SAMPLE_1S:
      for (float t = 0; t < 1; t += 0.1) {
        auto X = spline_.eval(t);
        auto Xd = spline_.evald(t);
        serialOut.printf("Spline Point: %.1f, %.4f, %.4f, %.4f, %.4f\n", t,
                         X[0], X[1], Xd[0], Xd[1]);
      }
    case '>': {  // debug - print interpolated X, Xdot at input time t
      UNWRAP_PARSE_CHECK(float t, parser.parseFloat('\n', &t));
      auto X = spline_.eval(t);
      auto Xd = spline_.evald(t);
      serialOut.printf("Spline Point: %.1f, %.4f, %.4f, %.4f, %.4f\n", t, X[0],
                       X[1], Xd[0], Xd[1]);
    }
    case '<': {  // debug - print the spline coefficients for segment `i`
      UNWRAP_PARSE_CHECK(int i, parser.parseInt('\n', &i));
      auto C = spline_.get_coeffs(i);
      serialOut.printf(
          "Spline Segment Coeffs: %d, %.3f, %.3f,  %.3f, %.3f, %.3f, %.3f,  "
          "%.3f, %.3f, %.3f, %.3f\n",
          i, spline_.get_breakpoint(i), spline_.get_breakpoint(i + 1),  //
          C[0][0], C[0][1], C[0][2], C[0][3], C[1][0], C[1][1], C[1][2],
          C[1][3]);
      break;
    }
    default:
      return false;
  }
  return true;
};
