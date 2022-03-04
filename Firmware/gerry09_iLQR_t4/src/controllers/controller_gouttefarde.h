#pragma once

#include <Metro.h>

#include "controller_simple.h"
#include "../constants.h"
#include "../kinematics.h"
#include "../pid.h"
// #include "../../trajectories/ATL_controller_1e4.h"
// #include "../../trajectories/ATL.h"
// #include "../../trajectories/ATL_filled.h"
// #include "../../trajectories/concentric_rects_controller_1e4.h"
#include "../../trajectories/concentric_diamonds2_output_lqg_1e1-1e2-1e2_0.1-0.0-0.002-0.06.h"
// static_assert((sizeof(painton) / sizeof(painton[0]) - 1) ==
//                   (sizeof(xffs) / sizeof(xffs[0])),
//               "Trajectories are not the same length");

class ControllerGouttefarde : public ControllerSimple {
 public:
  ControllerGouttefarde(const StateEstimatorInterface* state_estimator,
                        const Robot& robot)
      : ControllerSimple(state_estimator), robot_(robot), kinematics_(robot) {}

  // Datalogging
  std::pair<float, float> setpointVel() const override {
    return (state_ == RUNNING_TRAJ) ? desVel(trajTime_s())
                                    : std::make_pair(0.0f, 0.0f);
  }

  void update() override {
    ControllerSimple::update();
  }

  bool readSerial(AsciiParser parser, Stream& serialOut) override {
    UNWRAP_PARSE_CHECK(,parser.checkChar('K'));
    UNWRAP_PARSE_CHECK(char c, parser.getChar(&c));
    UNWRAP_PARSE_CHECK(float a, parser.parseFloat('\n', &a));
    switch (c) {
      case 'p':
        setKp(a);
        break;
      case 'i':
        setKi(a);
        break;
      case 'd':
        setKd(a);
        break;
      case 's':
        fs_ = a;
        break;
      case 'v':
        fv_ = a;
        break;
      case 'u':
        mu_ = a;
        break;
    }
    return true;
  }
  void writeSerial(Stream& serialOut) override {
    if (print_timer_.check() && (millis() - last_save_ms < 100)) {
      while (print_timer_.check())
        ;
      serialOut.printf("\t\t\t\t\t\t\t\t\t\t");
      serialOut.printf(":gains: %.1f %.1f %.1f %.2f %.2f %.2f\t", kp_, ki_, kd_,
                       fs_, fv_, mu_);
      // serialOut.printf(":PID output: %.3f %.3f %.3f %.3f\t", pid_output_[0],
      //                pid_output_[1], pid_output_[2], pid_output_[3]);
      // serialOut.printf(":Feedback force: %.3f %.3f\t", feedback_force_[0],
      //                feedback_force_[1]);
      // serialOut.printf(":Total force: %.3f %.3f\t", total_force_[0],
      //                total_force_[1]);
      serialOut.printf(":Torque: %.3f %.3f %.3f %.3f\n", torque_[0], torque_[1],
                     torque_[2], torque_[3]);
    }
  }
  Metro print_timer_{20};

 protected:
  static constexpr float dt = 0.01;
  const Robot& robot_;
  Kinematics kinematics_;

  std::pair<size_t, float> index_Remainder(float t) const;
  virtual Vector2 desPos(float t) const override;
  virtual Vector2 desVel(float t) const override;
  virtual float calcTorque(float t, uint8_t winchnum) const override;

  static constexpr size_t TRAJ_LEN = sizeof(LQG_GAINS) / sizeof(LQG_GAINS[0]);

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

 public:
  float fs_ = 0., fv_ = 0., mu_ = 0.;
  float kp_ = 5000., ki_ = 10., kd_ = 0.;
  Pid pid_[4]{Pid(kp_, ki_, kd_), Pid(kp_, ki_, kd_), Pid(kp_, ki_, kd_),
              Pid(kp_, ki_, kd_)};
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
// TODO(gerry): index_remainder is doing too much stuff - split this up
std::pair<size_t, float> ControllerGouttefarde::index_Remainder(float t) const {
  static int prev_color_ind = 0;
  size_t index = static_cast<int>(t / dt);
  // switch to "HOLD" if reached end of trajectory
  if (index >= TRAJ_LEN) {
    hold();  // TODO(gerry): make this not violate constness
    return {TRAJ_LEN - 1, dt};
  }
  float remainder = t - index * dt;
  return {index, remainder};
}
ControllerGouttefarde::Vector2 ControllerGouttefarde::desPos(float t) const {
  const auto& index_remainder = index_Remainder(t);
  return {LQG_GAINS[index_remainder.first].xff[1],
          LQG_GAINS[index_remainder.first].xff[2]};
}
ControllerGouttefarde::Vector2 ControllerGouttefarde::desVel(float t) const {
  const auto& index_remainder = index_Remainder(t);
  return {LQG_GAINS[index_remainder.first].vff[1],
          LQG_GAINS[index_remainder.first].vff[2]};
}
float ControllerGouttefarde::calcTorque(float t, uint8_t winchnum) const {
  if (winchnum >= 4) return 0.0f;
  // Setpoint
  const auto& index_remainder = index_Remainder(t);
  auto &k = index_remainder.first;
  const float(&xdes_)[3] = LQG_GAINS[k].xff;  // up to 10ms old
  const float(&vdes_)[3] = LQG_GAINS[k].vff;
  // First-order extrapolation
  float xdes[2], vdes[2];
  xdes[0] = xdes_[1] + vdes_[1] * index_remainder.second;
  xdes[1] = xdes_[2] + vdes_[2] * index_remainder.second;
  vdes[0] = vdes_[1];
  vdes[1] = vdes_[2];
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
  bool *tmp = &need_to_reset_pid_;
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
    float lerr = -ldes[i] - lengthCorrection(i, -robot_.len(i));
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
  const float(&vdes_prev)[3] = LQG_GAINS[k == 0 ? k : k - 1].vff;
  static constexpr float kMass = 1.0;
  ff_force_N[0] = (vdes[0] - vdes_prev[0]) / dt * kMass;
  ff_force_N[1] = (vdes[1] - vdes_prev[1]) / dt * kMass;
  float fc_N[2];
  matadd(feedback_force_N, ff_force_N, fc_N);
  save_total_force(fc_N);

  // Tension distribution
  // TODO(gerry): use better tension distribution algorithm
  float tensionTD_Nm[4];
  kinematics_.forceSolverPott(fc_N[0], fc_N[1], xdes[0], xdes[1], W,
                              tensionTD_Nm);
  // SerialD.printf("Tension Distribution: %.3f %.3f %.3f %.3f\n", tensionTD_Nm[0],
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
  clamp(&torque, 0.1, 1.2);
  return torque;
}
/************* END KEY FUNCTIONS **************/
