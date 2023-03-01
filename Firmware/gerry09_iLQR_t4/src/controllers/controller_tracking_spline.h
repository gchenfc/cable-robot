#pragma once

#include "controller_simple.h"
#include "../spline.h"
#include "../Vector.h"

/// @brief ControllerTracking takes setpoints/waypoints as Serial input and
/// updates desPos and desVel accordingly
class ControllerTrackingSpline : public ControllerSimple {
 protected:
  enum SplineExecutingState {
    NOMINAL_ON_PATH,  // Continue normally
    RETURN_TO_PATH,   // We're too far away from the setpoint: slowly move
                      // towards it
    HOLD_AT_DONE,     // We finished the spline, so just hold position
  };

 public:
  ControllerTrackingSpline(const StateEstimatorInterface* state_estimator)
      : ControllerSimple(state_estimator), spline_() {}

  static void print_name(Stream& serial) { serial.print("ControllerTrackingSpline"); }

  bool readSerial(AsciiParser parser, Stream& serialOut) override;

 protected:
  PPoly<750> spline_;
  float limit_left_ = 0.2, limit_right_ = 0.2, limit_up_ = 0.2,
        limit_down_ = 0.2;
  ControllerState prev_state_ = IDLE;
  SplineExecutingState spline_exec_state_ = RETURN_TO_PATH;
  PPoly<2, 2, 1> spline_return_;

  void myUpdate() override {
    if ((state_ == HOLD_TRAJ_BEGIN) ||
        ((state_ == RUNNING_TRAJ) && (prev_state_ != RUNNING_TRAJ))) {
      setReturnToPath();
    }
    if ((state_ == RUNNING_TRAJ) && (spline_exec_state_ == RETURN_TO_PATH)) {
      if (trajTime_s() > spline_return_.duration()) {
        // Hackily re-set time to 0
        state_ = HOLD_TRAJ_BEGIN;
        startTraj();  // TODO(gerry): this might mess up if we pause while
                      // returning to path.
        spline_exec_state_ = NOMINAL_ON_PATH;
      }
    }
    prev_state_ = state_;
  }

  template <int N>
  static Vector2 tuple2vector(std::array<float, N> x) {
    static_assert(std::tuple_size<decltype(x)>::value == 2,
                  "Not sure how to support splines not dim 2");
    return {std::get<0>(x), std::get<1>(x)};
  }

  // Set a path from current position to the next setpoint
  void setReturnToPath() { setReturnToPath(trajTime_s()); }
  void setReturnToPath(float t) {
    setReturnToPath(t, makeVector(state_estimator_->posEst()), spline_.eval(t));
  }
  void setReturnToPath(float t, Vector<2> x_cur, Vector<2> x_des) {
    static constexpr float kReturnSpeed = 0.1;
    auto dx = x_des - x_cur;
    float tmax = norm(dx) / kReturnSpeed;
    auto m = dx / tmax;

    spline_return_.reset();
    spline_return_.add_segment(t, {{{{0, std::get<0>(x_cur)}},  // Dummy segment
                                    {{0, std::get<1>(x_cur)}}}});
    spline_return_.add_segment(t + tmax,
                               {{{{std::get<0>(m), std::get<0>(x_cur)}},
                                 {{std::get<1>(m), std::get<1>(x_cur)}}}});
    spline_exec_state_ = RETURN_TO_PATH;
  }

  Vector2 desPos(float t) const override {
    Vector2 x = tuple2vector<2>((spline_exec_state_ == RETURN_TO_PATH)
                                    ? spline_return_.eval(t)
                                    : spline_.eval(t));
    clamp(&x.first, limit_left_, kWidth - limit_right_);
    clamp(&x.second, limit_down_, kHeight - limit_up_);
    return x;
  }
  Vector2 desVel(float t) const override {
    return tuple2vector<2>(spline_exec_state_ == RETURN_TO_PATH
                               ? spline_return_.evald(t)
                               : spline_.evald(t));
  }
};

bool ControllerTrackingSpline::readSerial(AsciiParser parser, Stream& serialOut) {
  if (ControllerSimple::readSerial(parser, serialOut)) return true;

  UNWRAP_PARSE_CHECK(, parser.checkChar('t'));
  UNWRAP_PARSE_CHECK(char cmd, parser.getChar(&cmd));

  std::pair<float, float> setpoint;
  float amt;
  switch (cmd) {
    case '&':
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
        serialOut.println("Error adding next segment of spline: out of space!");
      }
      break;
    case '-':
      spline_.reset();
      break;
    case '?':  // poll status
      serialOut.printf("Spline Status: %d %.3f %.3f\n", spline_exec_state_,
                       trajTime_s(), spline_.duration());
      break;
    case '#':  // debug - print number of total segments in the spline
      serialOut.printf("Spline number of segments: %d\n",
                       spline_.get_n_segments());
      break;
    case '*':  // debug - print trajectory from 0 to 1s
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
    case '%': {  // debug - compute and print spline_return_ trajectory
      setReturnToPath();
      float t1 = spline_return_.get_breakpoint(1),
            t2 = spline_return_.get_breakpoint(2);
      float dt = (t2 - t1) / 10;
      for (float t = t1; t < (t2 + dt); t += dt) {
        auto X = spline_return_.eval(t);
        serialOut.printf("Spline Return Point: %.2f, %.4f, %.4f\n", t, X[0],
                         X[1]);
      }
      break;
    }
    case 'L': {  // limits
      UNWRAP_PARSE_CHECK(char dir, parser.getChar(&dir));
      UNWRAP_PARSE_CHECK(, parser.parseFloat('\n', &amt));
      switch (dir) {
        case 'u':
          limit_up_ = amt;
          return true;
        case 'd':
          limit_down_ = amt;
          return true;
        case 'l':
          limit_left_ = amt;
          return true;
        case 'r':
          limit_right_ = amt;
          return true;
        default:
          return false;
      }
    }
    default:
      return false;
  }
  return true;
}
