#pragma once

#include "../communication/ascii_parser.h"
#include "../communication/cdpr_serial_commands.h"
#include "../state_estimators/state_estimator_interface.h"
#include "../Vector.h"
#include "../spline.h"
#include "setpoint_interface.h"

class Odrive;

/**
 * SetpointBasic defines some boilerplate code for SetpointInterface.  It could
 * be included in SetpointInterface, but figured it's just easier to read to
 * have it separate.
 */
class SetpointBasic : public SetpointInterface {
 public:
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
  virtual uint64_t time_us();
  float time_s() { return time_s(time_us()); }
  float time_s(uint64_t us) { return static_cast<float>(us) / 1e6; };

  /**************************** Setpoint wrappers ****************************/
  virtual X setpointPos() override;
  virtual V setpointVel() override;
  virtual A setpointAcc() override;
  /**************************** Implement these!!! ****************************/
  virtual X desPos(float t) = 0;
  virtual V desVel(float t) = 0;
  virtual A desAcc(float t) = 0;

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
  PPoly<2, 2, 1> travel_spline_;  // TODO: upgrade to include theta
  bool auto_advance_ = true;

  // Tunable parameters
  float switch_to_run_threshold_ = 0.1;
  float travel_speed_ = 0.1;
  X limits_min_ = {mountPoints[3][0] + 0.3f, mountPoints[3][1] + 0.3f, 0.};
  X limits_max_ = {mountPoints[1][0] - 0.3f, mountPoints[1][1] - 0.3f, 0.};

  // Convenience functions
  virtual X estimatedCurPos() const;
  virtual bool setState(State state);  // prep for next state
  static PPoly<2, 2, 1> calcTravelSpline(const X& start, const X& end,
                                         float speed = 0.1);

  static Vector<3> appendZeroTheta(const Vector<2>& x);
};

void SetpointBasic::update() {
  if (status_ == Status::NOMINAL) {
    switch (state_) {
      case State::HOLD:
        break;
      case State::RUNNING:
        break;
      case State::INTERMEDIATE_TRAVEL:
        if (time_s() >= travel_spline_.duration()) {
          setState(auto_advance_ ? State::RUNNING : State::HOLD);
          auto_advance_ = true;
        }
        break;
    }
  }
}

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
    case SetpointCommands::DEBUG_COMPUTE_AND_PRINT_TRAVEL_SPLINE_SAMPLED: {
      travel_spline_ = calcTravelSpline(
          estimatedCurPos(), desPos(time_s(t_paused_us_)), travel_speed_);
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

bool SetpointBasic::initialize() {
  if (status_ == Status::UNINITIALIZED) {
    hold_pos_ = estimatedCurPos();
    setState(State::HOLD);
    status_ = Status::NOMINAL;
    t_start_us_ = 0;
    t_travel_start_us_ = 0;
    t_paused_us_ = 0;
    return true;
  } else {
    return false;
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
  initialize();
  return true;
}

bool SetpointBasic::pause() { return setState(State::HOLD); }

bool SetpointBasic::advanceTo(float time_s) {
  if (state_ == State::RUNNING) {
    setState(State::HOLD);
    t_paused_us_ = time_s * 1e6;
    setState(State::INTERMEDIATE_TRAVEL);
    return true;
  } else {  // state is HOLD or INTERMEDIATE_TRAVEL
    t_paused_us_ = time_s * 1e6;
    return true;
  }
}

uint64_t SetpointBasic::time_us() {
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
  switch (state) {
    case State::HOLD:
      hold_pos_ = estimatedCurPos();
      if (state_ == State::RUNNING) {
        t_paused_us_ = time_us();
      }
      break;
    case State::RUNNING:
      if (norm(estimatedCurPos() - desPos(time_s(t_paused_us_))) >
          switch_to_run_threshold_) {
        return false;
      }
      t_start_us_ = micros() - t_paused_us_;
      break;
    case State::INTERMEDIATE_TRAVEL:
      travel_spline_ = calcTravelSpline(
          estimatedCurPos(), desPos(time_s(t_paused_us_)), travel_speed_);
      t_travel_start_us_ = micros();
      break;
    default:
      return false;
  }
  state_ = state;
  return true;
}

PPoly<2, 2, 1> SetpointBasic::calcTravelSpline(const X& start, const X& end,
                                               float speed) {
  Vector<3> dx_ = end - start;
  Vector<2> dx = {std::get<0>(dx_), std::get<1>(dx_)};
  float tmax = norm(dx) / speed;
  auto m = dx / tmax;

  PPoly<2, 2, 1> spline;
  spline.reset();
  spline.add_segment(0, {{{{0, std::get<0>(start)}},  // Dummy segment
                          {{0, std::get<1>(start)}}}});
  spline.add_segment(tmax, {{{{std::get<0>(m), std::get<0>(start)}},
                             {{std::get<1>(m), std::get<1>(start)}}}});
  return spline;
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
      return clamp(desPos(time_s()), limits_min_, limits_max_);
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
