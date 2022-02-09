#pragma once

#include <Metro.h>

#include "state_estimator_interface.h"
#include "../robot.h"

#include "../../trajectories/ATL_test_lqg.h"

static_assert(static_cast<uint64_t>(1000 * ESTIMATOR_DT) == 10,
              "expected dt to be 10ms");

class StateEstimatorKf : public StateEstimatorInterface {
 public:
  StateEstimatorKf(Robot &robot) : robot_(robot) {}

  void reset(uint64_t t_start) {
    tstart_us_ = t_start;
    state_ = {0, 0.0, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::fill(std::begin(most_recent_torques_), std::end(most_recent_torques_),
              0);
  }
  void save_torque(uint8_t winchnum, float torque) {
    most_recent_torques_[winchnum] = torque;
  }
  std::pair<float, float> posEst(uint64_t time_us = micros()) const override;
  std::pair<float, float> velEst(uint64_t time_us = micros()) const override;
  bool stateEst(uint64_t traj_time_s, float (&deltaXHat)[6]) const;

 protected:
  std::pair<size_t, float> index_Remainder(float t) const;
  float stateUpdate(float traj_time_s);

  struct State {
    size_t k;
    float tRemainder_s;
    float xHat[6];  // [deltax, deltav] at last discrete update
  } state_;

 private:
  Robot &robot_;
  uint64_t tstart_us_ = 0;
  float most_recent_torques_[4];

  static constexpr size_t TRAJ_LEN = sizeof(LQG_GAINS) / sizeof(LQG_GAINS[0]);
};

std::pair<size_t, float> StateEstimatorKf::index_Remainder(float t) const {
  size_t index = static_cast<int>(t / ESTIMATOR_DT);
  if (index >= (TRAJ_LEN)) {
    return {TRAJ_LEN - 1, ESTIMATOR_DT};
  }
  float remainder = t - index * ESTIMATOR_DT;
  return {index, remainder};
}

float StateEstimatorKf::stateUpdate(float traj_time_s) {
  //
  auto index_remainder = index_Remainder(traj_time_s);
  uint64_t k = index_remainder.first;
  float tRemainder_s = index_remainder.second;
  if (k == state_.k)
    return tRemainder_s;  // Don't need to update the state estimate

  // Need to update state estimate
  if (k != (state_.k + 1)) {
    // ERROR
    std::fill(std::begin(state_.xHat), std::end(state_.xHat), 0);
    return -1;
  }
  //
  const LqgGains &gains = LQG_GAINS[k];
  static float delta_u[4], delta_z[8];
  for (int i = 0; i < 4; ++i) {
    delta_z[i] = robot_.len(i) - gains.lff[i];
    delta_z[i + 4] = robot_.lenDot(i) - gains.ldotff[i];
    delta_u[i] = most_recent_torques_[i] - gains.uff[i];
  }

  // xHat = gains.Kx * xHat + gains.Ku * delta_u + gains.Kz * delta_z + k
  static float tmpX[6], tmpU[6], tmpZ[6], tmpSum[6];
  matmul(gains.Kx, state_.xHat, tmpX);
  matmul(gains.Ku, delta_u, tmpU);
  matmul(gains.Kz, delta_z, tmpZ);
  matadd(tmpX, tmpU, tmpSum);
  matadd(tmpSum, tmpZ, tmpSum);
  matadd(tmpSum, gains.k, tmpSum);

  // update state
  state_.k = k;
  state_.tRemainder_s = tRemainder_s;
  std::copy(std::begin(tmpSum), std::end(tmpSum), std::begin(state_.xHat));

  return tRemainder_s;
}

std::pair<float, float> StateEstimatorKf::posEst(uint64_t time_us) const {
  float tRemainder_s =
      stateUpdate(static_cast<float>(time_us - tstart_us_) / 1e6);

  // extrapolate
  std::pair<float, float> pos_now;
  pos_now.first = state_.xHat[1] + state_.xHat[4] * tRemainder_s;
  pos_now.second = state_.xHat[2] + state_.xHat[5] * tRemainder_s;

  // retract
  pos_now.first += LQG_GAINS[state_.k].xff[1];
  pos_now.second += LQG_GAINS[state_.k].xff[2];
  return pos_now;
};
std::pair<float, float> StateEstimatorKf::velEst(uint64_t time_us) const {
  float tRemainder_s =
      stateUpdate(static_cast<float>(time_us - tstart_us_) / 1e6);

  // extrapolate
  std::pair<float, float> vel_now;
  vel_now.first = state_.xHat[4];
  vel_now.second = state_.xHat[5];

  // retract
  vel_now.first += LQG_GAINS[state_.k].vff[1];
  vel_now.second += LQG_GAINS[state_.k].vff[2];
  return vel_now;
};
bool StateEstimatorKf::stateEst(uint64_t traj_time_s,
                                float (&deltaXHat)[6]) const {
  float tRemainder_s = stateUpdate(traj_time_s);
  if (tRemainder_s < 0) return false;
  for (int i = 0; i < 3; ++i) {
    deltaXHat[i] = state_.xHat[i] + state_.xHat[3 + i] * tRemainder_s;
    deltaXHat[3 + i] = state_.xHat[3 + i];
  }
  return true;
}
