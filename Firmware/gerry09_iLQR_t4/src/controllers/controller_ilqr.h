#pragma once

#include <Metro.h>

#include "controller_simple.h"
#include "../spray.h"
// #include "../../trajectories/ATL_controller_1e4.h"
// #include "../../trajectories/ATL.h"
// #include "../../trajectories/ATL_filled_output_controller_1e4.h"
// #include "../../trajectories/ATL_filled_output.h"
// #include "../../trajectories/2022-05-12_catherine/ATL_output.h"
// #include "../../trajectories/2022-05-12_catherine/ATL_output_controller.h"
#include "../../trajectories/2022-05-12_catherine/buzz_output.h"
#include "../../trajectories/2022-05-12_catherine/buzz_output_controller.h"
// #include "../../trajectories/concentric_diamonds_output_1mps_controller_1e4.h"
// #include "../../trajectories/concentric_diamonds_output_1mps.h"
// #include "../../trajectories/concentric_diamonds2_output_2mps_20mps2_controller_1e4.h"
// #include "../../trajectories/concentric_diamonds2_output_2mps_20mps2.h"
// #include "../../trajectories/ATL_filled.h"
// #include "../../trajectories/concentric_rects_controller_1e4.h"
// #include "/Users/gerry/Downloads/104494979_svg_output_3mps2_controller_full.h"
// #include "/Users/gerry/Downloads/104494979_svg_output_3mps2.h"
static_assert((sizeof(painton) / sizeof(painton[0]) - 1) ==
                  (sizeof(xffs) / sizeof(xffs[0])),
              "Trajectories are not the same length");

class ControllerIlqr : public ControllerSimple {
 public:
  ControllerIlqr(const StateEstimatorInterface* state_estimator, Spray& spray)
      : ControllerSimple(state_estimator), spray_(spray) {}

  // Datalogging
  std::pair<float, float> setpointVel() const override {
    return (state_ == RUNNING_TRAJ) ? desVel(trajTime_s())
                                    : std::make_pair(0.0f, 0.0f);
  }

 protected:
  static constexpr float dt = 0.01;
  Spray& spray_;

  std::pair<size_t, float> index_Remainder(float t) const;
  virtual Vector2 desPos(float t) const override;
  virtual Vector2 desVel(float t) const;
  virtual float calcTorque(float t, uint8_t winchnum) const override;
  void updatePaint(float t) override {
    if (t < 0) {
      spray_.setSpray(false);
      return;
    }
    size_t index = static_cast<int>(t / dt);
    if (index >= (sizeof(painton) / sizeof(painton[0]))) {
      spray_.setSpray(false);
    } else {
      spray_.setSpray(painton[index]);
    }
  }

  static constexpr size_t TRAJ_LEN = sizeof(xffs) / sizeof(xffs[0]);
  static_assert(TRAJ_LEN == (sizeof(vffs) / sizeof(vffs[0])),
                "Trajectory components have inconsistent size");
  static_assert(TRAJ_LEN == (sizeof(uffs) / sizeof(uffs[0])),
                "Trajectory components have inconsistent size");
  static_assert(TRAJ_LEN == (sizeof(Ks) / sizeof(Ks[0])),
                "Trajectory components have inconsistent size");
};

/************* KEY FUNCTIONS **************/
// TODO(gerry): index_remainder is doing too much stuff - split this up
std::pair<size_t, float> ControllerIlqr::index_Remainder(float t) const {
  static int prev_color_ind = 0;
  size_t index = static_cast<int>(t / dt);
  // switch to "HOLD" if reached end of trajectory
  if (index >= (sizeof(xffs) / sizeof(xffs[0]))) {
    hold();  // TODO(gerry): make this not violate constness
    return {TRAJ_LEN - 1, dt};
  }
  float remainder = t - index * dt;
  // Pause trajectory if point color changes
  if (painton[index]) {
    if (prev_color_ind != colorinds[index]) {
      stopTraj();  // TODO(gerry): don't violate const-ness
      prev_color_ind = colorinds[index];
      return {index, remainder};
    }
    prev_color_ind = colorinds[index];
  }
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

  // Safety
  // Note: this doesn't work for some reason, just be careful!
  const float xerr[2] = {xhat.first - xdes[0], xhat.second - xdes[1]};
  if (norm(xerr) > 0.4) {
    hold(); // TODO(gerry): don't violate const-ness
    return 0;
  }

  // Control calculation
  float error[4] = {xdothat.first - vdes[0], xdothat.second - vdes[1],
                    xhat.first - xdes[0], xhat.second - xdes[1]};
  float torque = dot(K, error) + uff;

  // Safety
  clamp(&torque, -0.1, 1.2);
  return torque;
}
/************* END KEY FUNCTIONS **************/
