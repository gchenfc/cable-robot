#pragma once

#include <Metro.h>

#include "controller_simple.h"
#include "../../trajectories/ATL_controller_1e2_1mps.h"

class ControllerGouttefarde : public ControllerSimple {
 public:
  ControllerGouttefarde(const StateEstimatorInterface* state_estimator,
                        const Robot& robot)
      : ControllerSimple(state_estimator), robot_(robot) {}

  // Datalogging
  std::pair<float, float> setpointVel() const override {
    return (state_ == RUNNING_TRAJ) ? desVel((micros() - tstart_us_) * 1e-6)
                                    : std::make_pair(0.0f, 0.0f);
  }

 protected:
  const Robot& robot_;

  std::pair<size_t, float> index_Remainder(float t) const;
  virtual Vector2 desPos(float t) const override;
  virtual Vector2 desVel(float t) const;
  virtual float calcTorque(float t, uint8_t winchnum) const override;

  static constexpr size_t TRAJ_LEN = sizeof(xffs) / sizeof(xffs[0]);
  static_assert(TRAJ_LEN == (sizeof(vffs) / sizeof(vffs[0])),
                "Trajectory components have inconsistent size");
  static_assert(TRAJ_LEN == (sizeof(uffs) / sizeof(uffs[0])),
                "Trajectory components have inconsistent size");
  static_assert(TRAJ_LEN == (sizeof(Ks) / sizeof(Ks[0])),
                "Trajectory components have inconsistent size");
};

/************* KEY FUNCTIONS **************/
std::pair<size_t, float> ControllerGouttefarde::index_Remainder(float t) const {
  static constexpr float dt = 0.01;
  size_t index = static_cast<int>(t / dt);
  if (index >= (sizeof(xffs) / sizeof(xffs[0]))) {
    stopTraj();  // TODO(gerry): make this not violate constness
    return {TRAJ_LEN - 1, dt};
  }
  float remainder = t - index * dt;
  return {index, remainder};
}
ControllerGouttefarde::Vector2 ControllerGouttefarde::desPos(float t) const {
  const auto& index_remainder = index_Remainder(t);
  return {xffs[index_remainder.first][0], xffs[index_remainder.first][1]};
}
ControllerGouttefarde::Vector2 ControllerGouttefarde::desVel(float t) const {
  const auto& index_remainder = index_Remainder(t);
  return {vffs[index_remainder.first][0], vffs[index_remainder.first][1]};
}

//output: feed forward force
float[2] inverseDynamics(ades) { 
  // return mass * ades;
  return 0;
}

//qdot is cable speed
float tensions_ff(float qdot) {
  return 0;
  // // static friction
  // float fs;
  // if (qdot > 0) {
  //   fs = -0.1 / r;
  // } else if (qdot == 0) {
  //   fs = 0;
  // } else {
  //   fs = 0.1 / r;
  // }
  // // viscous friction
  // fv = qdot * 0.1;
  // // motor inertia
  // float Tmotor = 1.0*qddot4des[i];
  // // sum
  // return fs + fv + Tmotor;
}
float[4] forceSolver(float[2] force) { return forceSolverPott(); }
// UNit test using this command in the `Firmware/gerry09_.../build` directory:
//  cmake ..
//  make -j8 test_runner_test && ./unit_test_simulator/test_runner_test
float ControllerGouttefarde::calcTorque(float t, uint8_t winchnum) const {
  if (winchnum >= 4) return 0.0f;
  // Setpoint
  const auto& index_remainder = index_Remainder(t);
  const float(&xdes)[2] = xffs[index_remainder.first];  // up to 10ms old
  const float(&vdes)[2] = vffs[index_remainder.first];
  float ades[2] =
      ? ? ?  // calculate acceleration desired (ades) = derivative of vdes

      // BEGIN GOUTTEFARDE
      float ldes[4],
        lact[4], tension_tmp[4];

  // fetch qm
  robot_.lens(lact);  // set lact
  // IK
  Kinematics.IK(xdes[0], xdes[1], ldes);  // set ldes IK is using xdes to solve for desired cable length
  // difference circle cross thingy
  lerror = ldes - lact;
  // PID 
  fPID = PID(lerror);
  // other circle cross thingy
  f = fPID + InverseDynamics(ades);
  // TD
  tensions = forceSolver(f);
  // other circle cross thingy
  tensions += tensions_ff(Kinematics.FKv(vdes));
  // tensions += tensions_ff(robot_.getLenDot(winchnum));

  torque = tensions[winchnum] * kR;

  clamp(&torque, -0.1, 1.2);
  return torque;
}
/************* END KEY FUNCTIONS **************/
