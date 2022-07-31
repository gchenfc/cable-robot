#pragma once

#include <Metro.h>
#include <tuple>

#include "controller_simple.h"
#include "../utils.h"
#include "../math.h"
#include "../spray.h"

#include "../../trajectories/concentric_diamonds2_output_lqg_1e1-1e2-1e2_0.1-0.0-0.002-0.06.h"

class ControllerShang20mmt : public ControllerSimple {
 public:
  using XVA = std::tuple<Vector<3>, Vector<3>, Vector<3>>;

 public:
  ControllerShang20mmt(const StateEstimatorInterface* state_estimator,
                       const Robot& robot, const Spray& spray)
      : ControllerSimple(state_estimator), robot_(robot), kinematics_(robot) {}

  void update() override { ControllerSimple::update(); }

  bool readSerial(AsciiParser parser, Stream& serialOut) override {
    if (parser.checkChar('K')) {
      UNWRAP_PARSE_CHECK(char c, parser.getChar(&c));
      UNWRAP_PARSE_CHECK(float a, parser.parseFloat('\n', &a));

#define MAKE_CASE(code, lhs) \
  case code:                 \
    lhs = a;                 \
    return true;

      switch (c) {
        MAKE_CASE('D', D_)
        MAKE_CASE('P', P_)
        MAKE_CASE('s', Ks_)
        MAKE_CASE('c', Kc_)
        MAKE_CASE('I', LambdaI_)
        MAKE_CASE('v', LambdaFv_)
        MAKE_CASE('f', LambdaFs_)
        MAKE_CASE('G', Gamma_)
        MAKE_CASE('<', minTension_)
        MAKE_CASE('=', midTension_)
        MAKE_CASE('>', maxTension_)
      }
#undef MAKE_CASE
    }
    return false;
  }
  void writeSerial(Stream& serialOut) override {}

  bool reset() override {
    int_c_ = Vector<4>();
    // winch parameters
    const float i = 19.6 * 1e-6 / kR,  // kg.m
        fs = 0.12 / kR,                // N
        fv = 0.002 / kR;               // N.s
    I_ = Vector<4>({i, i, i, i});
    Fv_ = Vector<4>({fv, fv, fv, fv});
    Fs_ = Vector<4>({fs, fs, fs, fs});
    // EE dynamics parameters
    M_ = Vector<2>({0.727, 0.727});
    C_ = Vector<2>({0.0, 0.0});
    G_ = Vector<2>({0.0, -9.8 * 0.727});
    return true;
  }

 protected:
  const Robot& robot_;
  Kinematics kinematics_;

  std::pair<size_t, float> index_Remainder(float t) const;
  virtual Vector2 desPos(float t) const override;
  virtual Vector2 desVel(float t) const override;

  XVA des(float t) const;
  virtual float calcTorque(float t, uint8_t winchnum) const override;

  // Control parameters
  float D_ = 0.8, P_ = 80;  // for some reason works better when D_ is negative?
  float Ks_ = 1, Kc_ = 3;
  float LambdaI_ = 1, LambdaFv_ = 4, LambdaFs_ = 2;
  float Gamma_ = 2.5;
  float minTension_ = 10, midTension_ = 50, maxTension_ = 90;
  Metro parameter_estimator_update_timer_{10}, debug_print_timer_{10};

  // Parameter Estimators
  Vector<4> int_c_{};
  Vector<4> I_, Fv_, Fs_;
  Vector<2> M_, C_;
  Vector<2> G_;
  void updateCSParams(const Vector<4>& ldot, const Vector<4>& lrdot,
                      const Vector<4>& lrddot, const Vector<4>& s);
  void updateTSParams(const Vector<2>& Xrddot, const Matrix<4, 2>& A,
                      const Vector<4>& s);

  static constexpr size_t TRAJ_LEN = sizeof(LQG_GAINS) / sizeof(LQG_GAINS[0]);
};

/************* KEY FUNCTIONS **************/
// TODO(gerry): index_remainder is doing too much stuff - split this up
std::pair<size_t, float> ControllerShang20mmt::index_Remainder(float t) const {
  static int prev_color_ind = 0;
  size_t index = static_cast<int>(t / CONTROLLER_DT);
  // switch to "HOLD" if reached end of trajectory
  if (index >= TRAJ_LEN) {
    hold();  // TODO(gerry): make this not violate constness
    return {TRAJ_LEN - 1, CONTROLLER_DT};
  }
  float remainder = t - index * CONTROLLER_DT;
  return {index, remainder};
}
ControllerShang20mmt::Vector2 ControllerShang20mmt::desPos(float t) const {
  const auto& index_remainder = index_Remainder(t);
  return {LQG_GAINS[index_remainder.first].xff[1],
          LQG_GAINS[index_remainder.first].xff[2]};
}
ControllerShang20mmt::Vector2 ControllerShang20mmt::desVel(float t) const {
  const auto& index_remainder = index_Remainder(t);
  return {LQG_GAINS[index_remainder.first].vff[1],
          LQG_GAINS[index_remainder.first].vff[2]};
}

ControllerShang20mmt::XVA ControllerShang20mmt::des(float t) const {
  const auto& index_remainder = index_Remainder(t);
  auto& k = index_remainder.first;
  Vector<3> xdes = LQG_GAINS[k].xff;  // up to 10ms old
  Vector<3> vdes = LQG_GAINS[k].vff;
  // First-order extrapolation
  xdes += vdes * index_remainder.second;
  // Finite difference approximation of acceleration
  Vector<3> vdes_next = LQG_GAINS[k >= TRAJ_LEN ? k : k + 1].vff;
  Vector<3> ades = (vdes_next - vdes) / CONTROLLER_DT;
  // Return
  return std::make_tuple(xdes, vdes, ades);
}

float ControllerShang20mmt::calcTorque(float t, uint8_t winchnum) const {
  if (winchnum >= 4) return 0.0f;

  // Task-space Setpoint
  Vector<3> xdes, vdes, ades;
  std::tie(xdes, vdes, ades) = des(t);
  // Joint-space Setpoint
  Vector<4> ldes, ldesdot, ldesddot;
  Matrix<4, 2> A;
  kinematics_.IK(xdes(1), xdes(2), ldes.asCVector());
  kinematics_.jacobian(xdes(1), xdes(2), A.asCMatrix());
  ldesdot = A * vdes.block<1, 0, 2, 1>();
  ldesddot = A * ades.block<1, 0, 2, 1>();  // TODO: coriolis
  // Measurement
  Vector<4> l, ldot;
  robot_.lens(l.asCVector());
  robot_.lenDots(ldot.asCVector());

  // Errors
  Vector<4> e = ldes - l;  // + = too short
  Vector<4> edot = ldesdot - ldot;
  Vector<4> es, c, esdot, cdot;
  for (int i = 0; i < 4; ++i) es(i) = e(i) - e((i + 1) % 4);
  for (int i = 0; i < 4; ++i) c(i) = es(i) - es((i + 3) % 4);  // + = too short
  for (int i = 0; i < 4; ++i) esdot(i) = edot(i) - edot((i + 1) % 4);
  for (int i = 0; i < 4; ++i) cdot(i) = esdot(i) - esdot((i + 3) % 4);
  int_c_ += c * CONTROLLER_DT;
  Vector<4> ec = e + D_ * int_c_;  // + = too short
  Vector<4> ecdot = edot + D_ * c;
  // Vector<4> ecddot = eddot + D_ * cdot;  // TODO: eddot
  Vector<4> s = ecdot + P_ * ec;  // + = too short

  // Reference CS trajectories
  Vector<4> lrdot = ldesdot + D_ * c + P_ * ec;
  Vector<4> lrddot = ldesddot + D_ * cdot + P_ * ecdot;

  // Reference TS trajectories
  Vector<2> Xrdot =
      vdes.block<1, 0, 2, 1>() + leastSquares(A, D_ * c + P_ * ec);
  Vector<2> Xrddot =
      ades.block<1, 0, 2, 1>() + leastSquares(A, D_ * cdot + P_ * ecdot);

  // feedforward
  Vector<4> u_ff = I_.elementwise() * lrddot + Fv_.elementwise() * lrdot +
                   Fs_.elementwise() * sign(ldot);

  // forward dynamics
  Vector<2> Fdes = M_.elementwise() * Xrddot + C_.elementwise() * Xrddot + G_;
  Vector<4> u_fd;
  kinematics_.forceSolverPott(Fdes(0), Fdes(1), A.T().asCMatrix(),
                              u_fd.asCVector(), midTension_);

  // total tension
  Vector<4> tension = u_ff - Ks_ * s - Kc_ * c + u_fd;
  Vector<4> tau = tension * kR;

  // update parameter estimators
  // if (parameter_estimator_update_timer_.check()) {
  //   updateCSParams(ldot, lrdot, lrddot, s);
  //   updateTSParams(Xrddot, A, s);
  //   while (parameter_estimator_update_timer_.check())
  //     ;
  // }

  // return
  float torque = tau(winchnum);
  clamp(&torque, minTension_ * kR, maxTension_ * kR);
  // clamp(&torque, 0.4, 0.9);

  // debug
  if (debug_print_timer_.check()) {
    Vector<4> fb = -Ks_ * s - Kc_ * c;
    Serial.printf(
        "%.1f %.1f %.1f %.1f\t-\t%.1f %.1f %.1f %.1f\t-\t%.2f %.2f\t-\t%.1f "
        "%.1f %.1f %.1f\n",
        u_ff(0), u_ff(1), u_ff(2), u_ff(3),  //
        fb(0), fb(1), fb(2), fb(3),          //
        Fdes(0), Fdes(1), tension(0), tension(1), tension(2), tension(3));
    while (debug_print_timer_.check())
      ;
  }

  return torque;
}

void ControllerShang20mmt::updateCSParams(const Vector<4>& ldot,
                                          const Vector<4>& lrdot,
                                          const Vector<4>& lrddot,
                                          const Vector<4>& s) {
  I_ += LambdaI_ * (lrddot.elementwise() * s);
  Fv_ += LambdaFv_ * (lrdot.elementwise() * s);
  auto signldot = sign(ldot);
  Fs_ += LambdaFs_ * (signldot.elementwise() * s);
}
void ControllerShang20mmt::updateTSParams(const Vector<2>& Xrddot,
                                          const Matrix<4, 2>& A,
                                          const Vector<4>& s) {
  M_ += Gamma_ * (Xrddot.elementwise() * leastSquares(A, s));
}
