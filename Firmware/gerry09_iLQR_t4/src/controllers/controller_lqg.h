#pragma once

#include <Metro.h>

#include "controller_simple.h"
#include "../state_estimators/state_estimator_kf.h"
#include "../../trajectories/ATL_test_lqg.h"
// #include "../../trajectories/ATL.h"

// static_assert((sizeof(painton) / sizeof(painton[0]) - 1) ==
//                   (sizeof(LQG_GAINS) / sizeof(LQG_GAINS[0])),
//               "Trajectories are not the same length");
static_assert(static_cast<uint64_t>(1000 * CONTROLLER_DT) == 10,
              "expected dt to be 10ms");

class ControllerLqg : public ControllerSimple {
 public:
  ControllerLqg(const StateEstimatorKf* state_estimator)
      : ControllerSimple(state_estimator),
        state_estimator_kf_(state_estimator) {}

  // Datalogging
  std::pair<float, float> setpointVel() const override {
    return (state_ == RUNNING_TRAJ) ? desVel(trajTime_s())
                                    : std::make_pair(0.0f, 0.0f);
  }

 protected:
  StateEstimatorKf* state_estimator_kf_;

  std::pair<size_t, float> index_Remainder(float t) const;
  virtual Vector2 desPos(float t) const override;
  virtual Vector2 desVel(float t) const;
  virtual float calcTorque(float t, uint8_t winchnum) const override;
  void updatePaint(float t) override {}

  bool startTraj() override {
    if (ControllerSimple::startTraj()) state_estimator_kf_->reset(tstart_us_);
  }

  static constexpr size_t TRAJ_LEN = sizeof(LQG_GAINS) / sizeof(LQG_GAINS[0]);
};

/************* KEY FUNCTIONS **************/
std::pair<size_t, float> ControllerLqg::index_Remainder(float t) const {
  size_t index = static_cast<int>(t / CONTROLLER_DT);
  if (index >= (TRAJ_LEN)) {
    hold();  // TODO: don't violate constness
    return {TRAJ_LEN - 1, CONTROLLER_DT};
  }
  float remainder = t - index * CONTROLLER_DT;
  return {index, remainder};
}
ControllerLqg::Vector2 ControllerLqg::desPos(float t) const {
  const auto& index_remainder = index_Remainder(t);
  return {LQG_GAINS[index_remainder.first].xff[1],
          LQG_GAINS[index_remainder.first].xff[2]};
}
ControllerLqg::Vector2 ControllerLqg::desVel(float t) const {
  const auto& index_remainder = index_Remainder(t);
  return {LQG_GAINS[index_remainder.first].vff[1],
          LQG_GAINS[index_remainder.first].vff[2]};
}
float ControllerLqg::calcTorque(float t, uint8_t winchnum) const {
  if (winchnum >= 4) return 0.0f;
  // state update
  static float deltaXHat[6];
  if (!state_estimator_kf_->stateEst(t, deltaXHat)) {
    hold();
    return 0.0f;
  }

  // Safety
  // Note: this safety measure doesn't work for some reason, just be careful!
  if (norm(deltaXHat) > 0.4) {
    hold();  // TODO(gerry): don't violate const-ness
    return 0;
  }

  // Setpoint
  const auto& index_remainder = index_Remainder(t);
  uint64_t k = index_remainder.first;
  float tRemainder = tRemainder;

  const float(&K)[6] = LQG_GAINS[k].K[winchnum];
  const float& uff = LQG_GAINS[k].uff[winchnum];

  // Control calculation
  float torque = dot(K, deltaXHat) + uff;

  // Safety
  clamp(&torque, -0.1, 1.2);

  // Return torque
  state_estimator_kf_->save_torque(winchnum, torque);
  return torque;
}
/************* END KEY FUNCTIONS **************/
