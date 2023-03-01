#pragma once

#include "../communication/ascii_parser.h"
#include "../communication/cdpr_serial_commands.h"
#include "../state_estimators/state_estimator_interface.h"
#include "../Vector.h"
#include "../spline.h"
#include "setpoint_interface.h"

class Odrive;

// Set the travel method by removing the "x" at the end of the define
#define TRAVEL_CONST_VELx
#define TRAVEL_TRAPEZOIDAL

/**
 * SetpointBasic defines some boilerplate code for SetpointInterface.  It could
 * be included in SetpointInterface, but figured it's just easier to read to
 * have it separate.
 *
 * SetpointBasic handles start/stop, timing, and travel strokes.
 */
class SetpointBasic : public SetpointInterface {
 public:
#if defined(TRAVEL_CONST_VEL)
  using TravelSpline = PPoly<2, 2, 1>;
#elif defined(TRAVEL_TRAPEZOIDAL)
  using TravelSpline = PPoly<4, 2, 2>;
#else
#error "Unsupported TRAVEL_### method"
#endif

  /******************************* Constructor *******************************/
  SetpointBasic(StateEstimatorInterface* state_estimator)
      : state_estimator_(state_estimator) {}

  /******************************** Common API ********************************/
  // virtual void setup() override;  // Called on bootup
  virtual void update() override;  // Called every loop
  virtual bool initialize() override;

  virtual bool readSerial(AsciiParser parser, Stream& serialOut) override;
  // virtual void writeSerial(Stream& serialOut) override;

  /**************************** State Control API ****************************/
  virtual bool travel() override;  // Does travel stroke but not run
  virtual bool start() override;
  virtual bool stop() override;
  virtual bool pause() override;
  virtual bool advanceTo(float time_s) override;
  virtual uint64_t time_us() const;
  float time_s() const { return time_s(time_us()); }
  float time_s(uint64_t us) const { return static_cast<float>(us) / 1e6; }
  bool isInLimits(const X& x, float tol = 0.0f) const {
    return inLimits(x, limits_min_, limits_max_, tol);
  }

  /**************************** Setpoint wrappers ****************************/
  virtual X setpointPos() override;
  virtual V setpointVel() override;
  virtual A setpointAcc() override;
  /**************************** Implement these!!! ****************************/
  virtual X desPos(float t) = 0;
  virtual V desVel(float t) = 0;
  virtual A desAcc(float t) = 0;
  virtual bool isDone(float t) = 0;

  /*************************** Protected Variables ***************************/
 protected:
  // State management
  StateEstimatorInterface* state_estimator_;

  // Timing
  uint64_t t_start_us_ = 0;
  uint64_t t_travel_start_us_ = 0;
  uint64_t t_paused_us_ = 0;

  // Variables used to smoothly move to start of trajectory
  X hold_pos_;
  TravelSpline travel_spline_;  // TODO: upgrade to include theta
  bool auto_advance_ = true;

  // Tunable parameters
  float switch_to_run_threshold_ = 0.1;
  float travel_speed_ = 0.1, travel_accel_ = 1.0;
  X limits_min_ = {{mountPoints[3][0] + kDefaultPaddingWidth,
                    mountPoints[3][1] + kDefaultPaddingHeight, 0.}};
  X limits_max_ = {{mountPoints[1][0] - kDefaultPaddingWidth,
                    mountPoints[1][1] - kDefaultPaddingHeight, 0.}};
  float limit_tol_ =
      kDefaultPaddingTol;  // When deciding to fault, how much to relax limits

  // Convenience functions
  virtual X estimatedCurPos() const;
  virtual bool setState(State state);  // prep for next state
  X desPosSafe(float t) { return clamp(desPos(t), limits_min_, limits_max_); }
  static TravelSpline calcTravelSpline(const X& start, const X& end,
                                       float max_speed = 0.1,
                                       float max_accel = 1.0,
                                       float cur_speed = 0.0);
  TravelSpline calcTravelSpline(const X& end) {
    return calcTravelSpline(estimatedCurPos(), end, travel_speed_,
                            travel_accel_, 0.0);
  }
  // If you call updateSetpoint, promise that immediately after you will do the
  // necessary updates to make desPos(0) = end
  bool updateSetpoint(const X& end);
  static Vector<3> appendZeroTheta(const Vector<2>& x);
};

bool SetpointBasic::readSerial(AsciiParser parser, Stream& serialOut) {
  if (SetpointInterface::readSerial(parser, serialOut)) return true;

  UNWRAP_PARSE_CHECK(, parser.checkChar(SerialPrefixes::SETPOINT));
  UNWRAP_PARSE_CHECK(char cmd, parser.getChar(&cmd));

  switch (cmd) {
    // state control
    case SetpointCommands::STOP0:
    case SetpointCommands::STOP1:
    case SetpointCommands::STOP2:
      serialOut.println("setpoint: STOP");
      stop();
      return true;
    case SetpointCommands::TRAVEL:
      serialOut.println(
          "setpoint: GO TO START TRAJECTORY (execute travel strokes)");
      travel();
      return true;
    case SetpointCommands::START:
      serialOut.println("setpoint: START TRAJECTORY");
      start();
      return true;
    case SetpointCommands::PAUSE:
      serialOut.println("setpoint: PAUSE TRAJECTORY");
      pause();
      return true;
    case SetpointCommands::RESET:
      serialOut.println("setpoint: RESET TRAJ");
      // advanceTo(0);
      stop();
      return true;
    case SetpointCommands::GOTO_TIME: {
      UNWRAP_PARSE_CHECK(float t, parser.parseFloat('\n', &t));
      serialOut.printf("setpoint: SET TO TRAJ TO %.3fs\n", t);
      advanceTo(t);
      return true;
    }
    case SetpointCommands::POLL_STATUS:
      serialOut.printf("setpoint: STATUS: %.3f %d %d %d\n", time_s(), state_,
                       status_, isDone(time_s()));
      return true;
    case SetpointCommands::DEBUG_COMPUTE_AND_PRINT_TRAVEL_SPLINE_SAMPLED: {
      travel_spline_ = calcTravelSpline(desPosSafe(time_s(t_paused_us_)));
      float t1 = travel_spline_.get_breakpoint(1),
            t2 = travel_spline_.get_breakpoint(2);
      float dt = (t2 - t1) / 10;
      for (float t = t1; t < (t2 + dt); t += dt) {
        auto X = travel_spline_.eval(t);
        serialOut.printf("setpoint: Travel Spline Point: %.2f, %.4f, %.4f\n", t,
                         X[0], X[1]);
      }
      return true;
    }
    case SetpointCommands::SET_SWITCH_TO_RUN_THRESHOLD: {
      UNWRAP_PARSE_CHECK(float threshold, parser.parseFloat('\n', &threshold));
      switch_to_run_threshold_ = threshold;
      return true;
    }
    case SetpointCommands::SET_TRAVEL_SPEED: {
      UNWRAP_PARSE_CHECK(float speed, parser.parseFloat('\n', &speed));
      travel_speed_ = speed;
      return true;
    }
    case SetpointCommands::SET_TRAVEL_ACCEL: {
      UNWRAP_PARSE_CHECK(float accel, parser.parseFloat('\n', &accel));
      travel_accel_ = accel;
      return true;
    }
    case SetpointCommands::SET_WORKSPACE_LIMITS_ABS: {
      UNWRAP_PARSE_CHECK(char dir, parser.getChar(&dir));
      UNWRAP_PARSE_CHECK(float amt, parser.parseFloat('\n', &amt));
      switch (dir) {
        case 'l':
          std::get<0>(limits_min_) = amt;
          return true;
        case 'r':
          std::get<0>(limits_max_) = amt;
          return true;
        case 'd':
          std::get<1>(limits_min_) = amt;
          return true;
        case 'u':
          std::get<1>(limits_max_) = amt;
          return true;
        case 'w':  // counter-clock-wise
          std::get<2>(limits_min_) = amt;
          return true;
        case 'c':  // clock-wise
          std::get<2>(limits_max_) = amt;
          return true;
        default:
          return false;
      }
    }
    case SetpointCommands::SET_WORKSPACE_LIMITS_REL: {
      UNWRAP_PARSE_CHECK(char dir, parser.getChar(&dir));
      UNWRAP_PARSE_CHECK(float amt, parser.parseFloat('\n', &amt));
      switch (dir) {
        case 'l':
          std::get<0>(limits_min_) = mountPoints[3][0] + amt;
          return true;
        case 'r':
          std::get<0>(limits_max_) = mountPoints[1][0] - amt;
          return true;
        case 'd':
          std::get<1>(limits_min_) = mountPoints[3][1] + amt;
          return true;
        case 'u':
          std::get<1>(limits_max_) = mountPoints[1][1] - amt;
          return true;
        case 'w':  // counter-clock-wise
          std::get<2>(limits_min_) = -amt;
          return true;
        case 'c':  // clock-wise
          std::get<2>(limits_max_) = amt;
          return true;
        default:
          return false;
      }
    }
    default:
      return false;
  }
}

void SetpointBasic::update() {
  if (!update_timer_.check()) return;
  if (status_ == Status::NOMINAL) {
    switch (state_) {
      case State::HOLD:
        break;
      case State::RUNNING:
        if (isDone(time_s())) stop();
        break;
      case State::INTERMEDIATE_TRAVEL:
        if (time_s() >= travel_spline_.duration()) {
          if (setState(auto_advance_ ? State::RUNNING : State::HOLD)) {
            auto_advance_ = true;
          }
        }
        break;
    }
  }
}

bool SetpointBasic::initialize() {
  if (status_ == Status::UNINITIALIZED) {
    hold_pos_ = estimatedCurPos();
    status_ = Status::NOMINAL;
    if (!setState(State::HOLD)) {
      status_ = Status::UNINITIALIZED;
      return false;
    }
    return true;
  } else {
    return true;
  }
}

bool SetpointBasic::travel() {
  auto_advance_ = false;
  return start();
}

bool SetpointBasic::start() {
  t_travel_start_us_ = micros() + t_paused_us_;
  return setState(State::INTERMEDIATE_TRAVEL);
}

bool SetpointBasic::stop() {
  status_ = Status::UNINITIALIZED;  // Dummy, just to get initialize() to run
  t_paused_us_ = 0;
  return initialize();
}

bool SetpointBasic::pause() {
  t_paused_us_ = time_us();
  return setState(State::HOLD);
}

bool SetpointBasic::advanceTo(float time_s) {
  if (state_ == State::RUNNING) {
    if (!setState(State::HOLD)) return false;
    t_paused_us_ = time_s * 1e6;
    if (!setState(State::INTERMEDIATE_TRAVEL)) return false;
    return true;
  } else {  // state is HOLD or INTERMEDIATE_TRAVEL
    t_paused_us_ = time_s * 1e6;
    return true;
  }
}

uint64_t SetpointBasic::time_us() const {
  switch (state_) {
    case State::HOLD:
      return t_paused_us_;
    case State::RUNNING:
      return micros() - t_start_us_;
    case State::INTERMEDIATE_TRAVEL:
      return micros() - t_travel_start_us_;
  }
  return 0;
}

bool SetpointBasic::setState(State state) {
  if (status_ != Status::NOMINAL) return false;
  switch (state) {
    case State::HOLD:
      hold_pos_ = (state_ == State::HOLD) ? estimatedCurPos() : setpointPos();
      if (!isInLimits(hold_pos_, limit_tol_)) {
        status_ = Status::UNINITIALIZED;
        return false;
      }
      break;
    case State::RUNNING:
      if (norm(estimatedCurPos() - desPosSafe(time_s(t_paused_us_))) >
          switch_to_run_threshold_) {
        return false;
      }
      t_start_us_ = micros() - t_paused_us_;
      break;
    case State::INTERMEDIATE_TRAVEL: {
      travel_spline_ = calcTravelSpline(desPosSafe(time_s(t_paused_us_)));
      t_travel_start_us_ = micros();
      break;
    }
    default:
      return false;
  }
  state_ = state;
  return true;
}

bool SetpointBasic::updateSetpoint(const SetpointBasic::X& end) {
  // This is basically just grabbing the relevant lines from setState above.
  // If you modify setState, you should probably also update this function.
  if (status_ != Status::NOMINAL) return false;
  auto end_safe = clamp(end, limits_min_, limits_max_);
  travel_spline_ = calcTravelSpline(setpointPos(), end_safe, travel_speed_,
                                    travel_accel_, norm(setpointVel()));
  t_travel_start_us_ = micros();
  state_ = State::INTERMEDIATE_TRAVEL;
  return true;
}

SetpointBasic::TravelSpline SetpointBasic::calcTravelSpline(const X& start,
                                                            const X& end,
                                                            float max_speed,
                                                            float max_accel,
                                                            float cur_speed) {
  TravelSpline spline;
  spline.reset();

  Vector<3> dx_ = end - start;
  Vector<2> dx = {std::get<0>(dx_), std::get<1>(dx_)};
  float d = norm(dx);

#if defined(TRAVEL_CONST_VEL)
  float tmax = d / max_speed;
  auto m = dx / std::max(tmax, 1e-6f);

  spline.add_segment(0, {{{{0, std::get<0>(start)}},  // Dummy segment
                          {{0, std::get<1>(start)}}}});
  spline.add_segment(tmax, {{{{std::get<0>(m), std::get<0>(start)}},
                             {{std::get<1>(m), std::get<1>(start)}}}});
  return spline;
#elif defined(TRAVEL_TRAPEZOIDAL)
  // Creates a trapezoidal velocity profile with max acceleration of `max_accel`
  // and max velocity of `max_speed`.
  auto dx_normalized = dx / std::max(d, 1e-6f);
  float t_accel = (max_speed - cur_speed) / max_accel;
  float d_accel = t_accel * (max_speed + cur_speed) / 2;
  float t_decel = max_speed / max_accel;
  float d_decel = t_decel * max_speed / 2;
  float min_d = cur_speed * (cur_speed / max_accel) / 2;

  // Helper function to add a segment to the spline
  // Of the form: x(t) = a*t^2 + b*t + c
  auto add_segment = [&spline, &dx_normalized, &start](float tmax, float a,
                                                       float b, float c) {
    auto A = a * dx_normalized;
    auto B = b * dx_normalized;
    auto C = c * dx_normalized;
    spline.add_segment(tmax, {{{{std::get<0>(A), std::get<0>(B),
                                 std::get<0>(C) + std::get<0>(start)}},
                               {{std::get<1>(A), std::get<1>(B),
                                 std::get<1>(C) + std::get<1>(start)}}}});
  };

  if (d < 1e-6) {  // "0" segments (already at destination)
    add_segment(0, 0, 0, 0);
    return spline;
  }
  if (d <= min_d) {  // "1" segments (cannot even decelerate to a stop)
    // This is bad, we can't even decelerate to a stop.
    // Just violate deceleration limits.
    // d = 0.5.t.vcur, so t = 2.d/vcur
    float t = 2 * d / std::max(cur_speed, 1e-6f);
    float accel = -cur_speed / std::max(t, 1e-6f);
    add_segment(t, 0.5 * accel, cur_speed, 0);
    return spline;
  } else if (d <= (d_accel + d_decel)) {  // "2" segments: triangular profile
    /* If the distance is too short, we need to switch to a triangular profile
     * d = (v0 + v) * (v - v0) / a / 2 + (vf + v) * (v - vf) / a / 2
     *   = ((v0 + v) * (v - v0) + (vf + v) * (v - vf)) / a / 2
     *   = (v^2 - v0^2 + v^2 - vf^2) / a / 2
     * 2*v^2 = 2*a*d + (v0^2 + vf^2)
     * also, vf = 0
     */
    float vmax = sqrt(max_accel * d + cur_speed * cur_speed / 2);
    float t_accel = (vmax - cur_speed) / max_accel;
    float t_decel = vmax / max_accel;

    add_segment(0, 0, 0, 0);  // Dummy segment
    // First accelerate: x = 0.5.a.t^2 + v0.t + 0
    add_segment(t_accel, 0.5 * max_accel, cur_speed, 0);
    auto x = 0.5 * max_accel * t_accel * t_accel + cur_speed * t_accel;
    // Now decelerate: x = -0.5.a.t^2 + v.t + x0
    add_segment(t_accel + t_decel, -0.5 * max_accel, vmax, x);
    return spline;
  } else {  // "3" segments: Trapezoidal profile
    float t_accel = (max_speed - cur_speed) / max_accel;
    float t_decel = max_speed / max_accel;
    float d_accel = 0.5 * max_accel * t_accel * t_accel + cur_speed * t_accel;
    float d_decel = 0.5 * max_accel * t_decel * t_decel;
    float t_mid = (d - d_accel - d_decel) / max_speed;
    float t1 = t_accel, t2 = t_accel + t_mid, t3 = t_accel + t_mid + t_decel;
    add_segment(0, 0, 0, 0);             // Dummy segment
    add_segment(t1, 0.5 * max_accel, cur_speed, 0);  // First accelerate
    add_segment(t2, 0, max_speed, d_accel);  // Then constant velocity
    add_segment(t3, -0.5 * max_accel, max_speed, d - d_decel);  // Then decelerate
    return spline;
  }
#else
#error "Unsupported TRAVEL_### mode"
#endif
}

Vector<3> SetpointBasic::estimatedCurPos() const {
  auto x = state_estimator_->posEst();
  return {x.first, x.second, state_estimator_->thetaEst()};
}

SetpointInterface::X SetpointBasic::setpointPos() {
  initialize();
  switch (state_) {
    case State::HOLD:
      return hold_pos_;
    case State::RUNNING:
      return desPosSafe(time_s());
    case State::INTERMEDIATE_TRAVEL:
      return appendZeroTheta(travel_spline_.eval(time_s()));
  }
  return kDefaultPos<3>();
}

SetpointInterface::V SetpointBasic::setpointVel() {
  switch (state_) {
    case State::HOLD:
      return kZero<3>();
    case State::RUNNING:
      return desVel(time_s());
    case State::INTERMEDIATE_TRAVEL:
      return appendZeroTheta(travel_spline_.evald(time_s()));
  }
  return kZero<3>();
}

SetpointInterface::A SetpointBasic::setpointAcc() {
  switch (state_) {
    case State::HOLD:
      return kZero<3>();
    case State::RUNNING:
      return desAcc(time_s());
    case State::INTERMEDIATE_TRAVEL:
      return appendZeroTheta(travel_spline_.evaldd(time_s()));
  }
  return kZero<3>();
}

Vector<3> SetpointBasic::appendZeroTheta(const Vector<2>& x) {
  return Vector<3>{x[0], x[1], 0.};
}
