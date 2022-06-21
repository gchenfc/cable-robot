#pragma once

#include <Metro.h>

#include "controller_tracking.h"
#include "../constants.h"
#include "../kinematics.h"
#include "../pid.h"

/******** DEFAULT GAINS AND PARAMETERS **********/
#if defined(KLAUS) || defined(DFL)
static constexpr float kKp = 15000, kKi = 15, kKd = 30,  // gains
    kTMin_N = 8, kTMid_N = 54, kTMax_N = 100;            // tensions
#endif
#ifdef HYDROPONICS
static constexpr float kKp = 30000, kKi = 50, kKd = 50,  // gains
    kTMin_N = 5, kTMid_N = 50, kTMax_N = 140;            // tensions
#endif

/* ControllerGouttefardeTracking
 * Tracking controller using Gouttefarde's dual-space FF controller.
 * Uses Pott's tension distribution algorithm.
 */
class ControllerGouttefardeTracking : public ControllerTracking {
 public:
  ControllerGouttefardeTracking(const StateEstimatorInterface* state_estimator,
                                const Robot& robot)
      : ControllerTracking(state_estimator),
        robot_(robot),
        kinematics_(robot) {}

  bool readSerial(AsciiParser parser, Stream& serialOut) override {
    if (ControllerTracking::readSerial(parser, serialOut)) return true;

    {
      AsciiParser parser_gains = parser;
      UNWRAP_PARSE_CHECK(, parser_gains.checkChar('K'));
      UNWRAP_PARSE_CHECK(char c, parser_gains.getChar(&c));
      UNWRAP_PARSE_CHECK(float a, parser_gains.parseFloat('\n', &a));
      switch (c) {
        case 'p':
          setKp(a);
          return true;
        case 'i':
          setKi(a);
          return true;
        case 'd':
          setKd(a);
          return true;
        case 's':
          fs_ = a;
          return true;
        case 'v':
          fv_ = a;
          return true;
        case 'u':
          mu_ = a;
          return true;
        case 'm':
          midTension_ = a;
          return true;
        case 'M':
          maxTension_ = a;
          return true;
        case 'L':
          minTension_ = a;
          return true;
      }
    }
    {
      AsciiParser parser_print_debug = parser;
      UNWRAP_PARSE_CHECK(, parser_print_debug.checkChar('p'));
      UNWRAP_PARSE_CHECK(char c, parser_print_debug.getChar(&c));
      UNWRAP_PARSE_CHECK(int a, parser_print_debug.parseInt('\n', &a));
      switch (c) {
        case 'g':
          print_gains_ = a > 0;
          return true;
        case 'p':
          print_pid_ = a > 0;
          return true;
        case '<':
          print_force_fb_ = a > 0;
          return true;
        case 'f':
          print_force_ = a > 0;
          return true;
        case 't':
          print_torque_ = a > 0;
          return true;
        case 'i':
          print_timer_.interval(a);
          return true;
      }
    }
    return false;
  }
  void writeSerial(Stream& serialOut) override {
    if (print_timer_.check() && (millis() - last_save_ms < 100)) {
      if (print_gains_) {
        serialOut.printf(":gains: %.1f %.1f %.1f %.2f %.2f %.2f\t", kp_, ki_,
                         kd_, fs_, fv_, mu_);
      }
      if (print_pid_) {
        serialOut.printf(":PID output: %.3f %.3f %.3f %.3f\t", pid_output_[0],
                         pid_output_[1], pid_output_[2], pid_output_[3]);
      }
      if (print_force_fb_) {
        serialOut.printf(":Feedback force: %.3f %.3f\t", feedback_force_[0],
                         feedback_force_[1]);
      }
      if (print_force_) {
        serialOut.printf(":Total force: %.3f %.3f\t", total_force_[0],
                         total_force_[1]);
      }
      if (print_torque_) {
        serialOut.printf(":Torque: %.3f %.3f %.3f %.3f\n", torque_[0],
                         torque_[1], torque_[2], torque_[3]);
      }
      if (print_gains_ || print_pid_ || print_force_fb_ || print_force_ ||
          print_torque_) {
        serialOut.println();
      }
    }
  }

 protected:
  Metro print_timer_{1000, 1};
  bool print_gains_ = false, print_pid_ = false, print_force_fb_ = false,
       print_force_ = false, print_torque_ = false;
  static constexpr float dt = 0.01;
  const Robot& robot_;
  Kinematics kinematics_;

  virtual Vector2 desVel(float t) const override;
  virtual float calcTorque(float t, uint8_t winchnum) const override;

 public:
  float pid_output_[4], feedback_force_[2], total_force_[2], torque_[4];
  uint64_t last_save_ms = 0;
  bool need_to_reset_pid_ = true;
  void save_pid_output(const float (&in)[4]) {
    std::copy(std::begin(in), std::end(in), std::begin(pid_output_));
    last_save_ms = millis();
  }
  void save_feedback_force(const float (&in)[2]) {
    std::copy(std::begin(in), std::end(in), std::begin(feedback_force_));
    last_save_ms = millis();
  }
  void save_total_force(const float (&in)[2]) {
    std::copy(std::begin(in), std::end(in), std::begin(total_force_));
    last_save_ms = millis();
  }
  void save_torque(const float (&in)[4]) {
    std::copy(std::begin(in), std::end(in), std::begin(torque_));
    last_save_ms = millis();
  }

 protected:
  float fs_ = 0., fv_ = 0., mu_ = 0.;
  float kp_ = kKp, ki_ = kKi, kd_ = kKd;
  float minTension_ = kTMin_N, midTension_ = kTMid_N, maxTension_ = kTMax_N;
  Pid pid_[4]{Pid(kp_, ki_, kd_), Pid(kp_, ki_, kd_), Pid(kp_, ki_, kd_),
              Pid(kp_, ki_, kd_)};

 public:
  void setKp(float kp) {
    kp_ = kp;
    for (Pid& pid : pid_) {
      pid.setKp(kp);
    }
  }
  void setKi(float ki) {
    ki_ = ki;
    for (Pid& pid : pid_) {
      pid.setKi(ki);
    }
  }
  void setKd(float kd) {
    kd_ = kd;
    for (Pid& pid : pid_) {
      pid.setKd(kd);
    }
  }
};

/************* KEY FUNCTIONS **************/
ControllerGouttefardeTracking::Vector2 ControllerGouttefardeTracking::desVel(
    float t) const {
  // float x2, y2;
  // towards(1e-3 * SPEED,                       //
  //         cur_.first, cur_.second,            // current
  //         setpoint_.first, setpoint_.second,  // target
  //         &x2, &y2);                          // new
  // return {(x2 - cur_.first) / 1e-3, (y2 - cur_.second) / 1e-3};
  return {0., 0.};
}
float ControllerGouttefardeTracking::calcTorque(float t,
                                                uint8_t winchnum) const {
  if (winchnum >= 4) return 0.0f;
  // Setpoint
  const auto& xdes_ = desPos(t);
  const auto& vdes_ = desVel(t);
  const float xdes[2] = {xdes_.first, xdes_.second};
  const float vdes[2] = {vdes_.first, vdes_.second};

  // SerialD.printf("xdes = %.3f %.3f, vdes = %.3f %.3f\n", xdes[0], xdes[1],
  //                vdes[0], vdes[1]);

  // IK
  float ldes[4];
  kinematics_.IK(xdes[0], xdes[1], ldes);
  // const float(&lff)[4] = LQG_GAINS[k].lff;
  // SerialD.printf("ldes = %.3f %.3f %.3f %.3f, lff = %.3f %.3f %.3f %.3f\n",
  //                ldes[0], ldes[1], ldes[2], ldes[3], lff[0], lff[1], lff[2],
  //                lff[3]);

  // PID
  bool* tmp = &need_to_reset_pid_;
  if (t < 0.01) {
    if (need_to_reset_pid_) {
      for (int i = 0; i < 4; ++i) {
        pid_[i].reset();
      }
      *tmp = false;
    }
  } else {
    *tmp = true;
  }

  // float feedback_torque_Nm[4];
  float feedback_tension_N[4];
  for (int i = 0; i < 4; ++i) {
    float lerr = ldes[i] - robot_.len(i);
    feedback_tension_N[i] = pid_[i].update(lerr);
  }

  save_pid_output(feedback_tension_N);

  // WR^-1
  // float feedback_tension_N[4];
  // for (int i = 0; i < 4; ++i) {
  //   feedback_tension_N = feedback_torque_Nm / kR;
  // }
  float W[2][4];
  kinematics_.wrenchMatrix(W);
  float feedback_force_N[2];
  matmul(W, feedback_tension_N, feedback_force_N);
  save_feedback_force(feedback_force_N);

  // feedforward force
  // TODO(gerry): SE(2)
  float ff_force_N[2];
  // TODO(gerry): save previous vdes
  // const float(&vdes_prev)[3] = LQG_GAINS[k == 0 ? k : k - 1].vff;
  static constexpr float kMass = 1.0;
  ff_force_N[0] = 0;  //(vdes[0] - vdes_prev[0]) / dt * kMass;
  ff_force_N[1] = 0;  //(vdes[1] - vdes_prev[1]) / dt * kMass;
  float fc_N[2];
  matadd(feedback_force_N, ff_force_N, fc_N);
  save_total_force(fc_N);

  // Tension distribution
  // TODO(gerry): use better tension distribution algorithm
  float tensionTD_Nm[4];
  kinematics_.forceSolverPott(fc_N[0], fc_N[1], xdes[0], xdes[1], W,
                              tensionTD_Nm, midTension_);
  // SerialD.printf("Tension Distribution: %.3f %.3f %.3f %.3f\n",
  //                tensionTD_Nm[0],
  //                tensionTD_Nm[1], tensionTD_Nm[2],
  //                tensionTD_Nm[3]);

  // feedforward torques
  float ldotdes[4];
  float J[4][2];
  kinematics_.jacobian(J);
  matmul(J, vdes, ldotdes);
  float taum_Nm[4];
  for (int i = 0; i < 4; ++i) {
    taum_Nm[i] =
        (tensionTD_Nm[i] - ldotdes[i] * fv_ - tanh(mu_ * ldotdes[i]) * fs_) *
        kR;
  }
  save_torque(taum_Nm);

  // Safety & return
  float torque = taum_Nm[winchnum];
  clamp(&torque, minTension_ * kR, maxTension_ * kR);
  return torque;
}
/************* END KEY FUNCTIONS **************/
