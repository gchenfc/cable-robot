#pragma once

#include "../communication/ascii_parser.h"
#include "../Vector.h"
#include "../spline.h"
#include "setpoint_basic.h"

/**
 * SetpointWaypoints implements a waypoint tracking setpoint calculator.
 * It is NOT a circular buffer, but should be good enough if you send one stroke
 * at a time.
 */
class SetpointWaypoints : public SetpointBasic {
 public:
  using SetpointBasic::SetpointBasic;

  static void print_name(Stream& serialOut) {
    serialOut.print("SetpointWaypoints");
  }

  /******************************** Common API ********************************/
  virtual void setup() override {
    SetpointBasic::setup();
    spline_.reset();
    spline_.set_default_pos(defaultPos());
  }

  virtual bool readSerial(AsciiParser parser, Stream& serialOut) override;

  virtual X desPos(float t) override { return spline_.eval(t); }
  virtual V desVel(float t) override { return spline_.evald(t); }
  virtual A desAcc(float t) override { return spline_.evaldd(t); }
  virtual bool isDone(float t) override { return t >= spline_.duration(); }

 protected:
  PPoly<1000, 3, 1> spline_;
  float max_speed_ = 0.1;
  float max_accel_ = 0.5;

  bool pushWaypoint(const X& waypoint);
};

bool SetpointWaypoints::pushWaypoint(const X& waypoint) {
  if (spline_.get_n_segments() == 0) {
    // auto xcur = spline_.eval(spline_.duration());
    spline_.add_segment(0, {{{{0, std::get<0>(waypoint)}},
                             {{0, std::get<1>(waypoint)}},
                             {{0, std::get<2>(waypoint)}}}});
    return true;
  }
  // check if the distance between the proposed waypoint and the last waypoint
  // is too small
  auto start = spline_.eval(spline_.duration());
  auto dx = waypoint - start;
  float dist = norm(dx);
  if (dist < 0.001) return false;
  float tmax = dist / max_speed_;
  dx = dx / tmax;
  return spline_.add_segment(spline_.duration() + tmax,
                             {{{{std::get<0>(dx), std::get<0>(start)}},
                               {{std::get<1>(dx), std::get<1>(start)}},
                               {{std::get<2>(dx), std::get<2>(start)}}}});
}

bool SetpointWaypoints::readSerial(AsciiParser parser, Stream& serialOut) {
  if (SetpointBasic::readSerial(parser, serialOut)) return true;

  UNWRAP_PARSE_CHECK(, parser.checkChar(SerialPrefixes::SETPOINT));
  UNWRAP_PARSE_CHECK(char cmd, parser.getChar(&cmd));

  switch (cmd) {
    case SetpointCommands::WAYPOINT_CLEAR:
      serialOut.println("waypoints reset.");
      spline_.reset();
      spline_.set_default_pos(defaultPos());
      break;
    case SetpointCommands::WAYPOINT_ADD:
      UNWRAP_PARSE_CHECK(float x, parser.parseFloat(',', &x));
      UNWRAP_PARSE_CHECK(float y, parser.parseFloat(',', &y));
      UNWRAP_PARSE_CHECK(float th, parser.parseFloat('\n', &th));
      if (!pushWaypoint({{x, y, th}})) {
        serialOut.println(
            "Waypoint rejected: too close to previous waypoint or out of "
            "space.");
      } else {
        serialOut.printf("Waypoint added: %.3f, %.3f, %.3f\n", x, y, th);
      }
      break;
    case SetpointCommands::SET_WAYPOINT_SPEED:
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &max_speed_));
      serialOut.printf("Waypoint set max_speed_ to %.3f\n", max_speed_);
      break;
    case SetpointCommands::SET_WAYPOINT_ACCEL:
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &max_accel_));
      serialOut.printf(
          "Waypoint set max_accel_ to %.3f.  Warning: acceleration isn't "
          "currently being used\n",
          max_accel_);
      break;
    case SetpointCommands::WAYPOINTS_PRINT_NUM_WAYPOINTS:
      serialOut.printf("Waypoint: %d waypoints\n", spline_.get_n_segments());
      break;
    case SetpointCommands::WAYPOINTS_PRINT_WAYPOINT: {
      UNWRAP_PARSE_CHECK(int i, parser.parseInt('\n', &i));
      if (i < 0 || i >= spline_.get_n_segments()) {
        serialOut.printf("Waypoint: invalid index %d\n", i);
        break;
      }
      auto coeffs = spline_.get_coeffs(i);
      serialOut.printf("Waypoint %d: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", i,
                       coeffs.at(0).at(0), coeffs.at(0).at(1),
                       coeffs.at(1).at(0), coeffs.at(1).at(1),
                       coeffs.at(2).at(0), coeffs.at(2).at(1));
      break;
    }
    default:
      return false;
  }
  return true;
};
