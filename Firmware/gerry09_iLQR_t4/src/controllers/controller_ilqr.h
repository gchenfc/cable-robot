#pragma once

#include <Metro.h>

#include "controller_simple.h"
#include "../../trajectories/ATL_controller_1e2_1mps.h"

class ControllerIlqr : public ControllerSimple {
 public:
  ControllerIlqr(const StateEstimatorInterface* state_estimator)
      : ControllerSimple(state_estimator) {}

  // Datalogging
  std::pair<float, float> setpointVel() const override {
    return (state_ == RUNNING_TRAJ) ? desVel((micros() - tstart_us_) * 1e-6)
                                    : std::make_pair(0.0f, 0.0f);
  }

 protected:
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
std::pair<size_t, float> ControllerIlqr::index_Remainder(float t) const {
  static constexpr float dt = 0.01;
  size_t index = static_cast<int>(t / dt);
  if (index >= (sizeof(xffs) / sizeof(xffs[0]))) {
    stopTraj(); // TODO(gerry): make this not violate constness
    return {TRAJ_LEN - 1, dt};
  }
  float remainder = t - index * dt;
  return {index, remainder};
}
ControllerIlqr::Vector2 ControllerIlqr::desPos(float t) const {
  const auto& index_remainder = index_Remainder(t);
  return {xffs[index_remainder.first][0], xffs[index_remainder.first][1]};
}
ControllerIlqr::Vector2 ControllerIlqr::desVel(float t) const {
  const auto& index_remainder = index_Remainder(t);
  return {vffs[index_remainder.first][0], vffs[index_remainder.first][1]};
}
float ControllerIlqr::calcTorque(float t, uint8_t winchnum) const {
  if (winchnum >= 4) return 0.0f;
  // Setpoint
  const auto& index_remainder = index_Remainder(t);
  const float(&xdes_)[2] = xffs[index_remainder.first];  // up to 10ms old
  const float(&vdes)[2] = vffs[index_remainder.first];
  const float& uff = uffs[index_remainder.first][winchnum];
  const float(&K)[4] = Ks[index_remainder.first][winchnum];
  // First-order extrapolation
  float xdes[2];
  xdes[0] = xdes_[0] + vdes[0] * index_remainder.second;
  xdes[1] = xdes_[1] + vdes[1] * index_remainder.second;

  // Current state
  const Vector2& xhat = state_estimator_->posEst();
  const Vector2& xdothat = state_estimator_->velEst();

  // Control calculation
  float error[4] = {xdothat.first - vdes[0], xdothat.second - vdes[1],
                    xhat.first - xdes[0], xhat.second - xdes[1]};
  float torque = dot(K, error) + uff;

  // Safety
  clamp(&torque, -0.1, 1.2);
  return torque;
}
/************* END KEY FUNCTIONS **************/
