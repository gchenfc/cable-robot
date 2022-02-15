#pragma once

#include <Metro.h>

#include "state_estimator_interface.h"
#include "../robot.h"

// #include "../../trajectories/ATL_lqg_1e4-1e4-1e4.h"
// #include "../../trajectories/ATL_lqg_1e0-1e2-1e2_0.1-0.1-0.05.h"
// #include "../../trajectories/ATL_lqg_1e2-1e4-1e4_0.1-0.0-0.05-0.05.h"
#include "../../trajectories/ATLinf_lqg_1e2-1e4-1e4_0.1-0.0-0.05-0.05.h"

static_assert(static_cast<uint64_t>(1000 * ESTIMATOR_DT) == 10,
              "expected dt to be 10ms");

class StateEstimatorKf : public StateEstimatorInterface {
 public:
  StateEstimatorKf(Robot &robot) : robot_(robot) {}

  void reset(uint64_t t_start) {
    tstart_us_ = t_start;
    state_ = {0,
              0.0,
              {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
              {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
              {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::fill(std::begin(most_recent_torques_), std::end(most_recent_torques_),
              0);
  }
  void save_torque(uint8_t winchnum, float torque) {
    most_recent_torques_[winchnum] = torque;
  }
  std::pair<float, float> posEst(uint64_t time_us = micros()) const override;
  std::pair<float, float> velEst(uint64_t time_us = micros()) const override;
  bool stateEst(float traj_time_s, float (&deltaXHat)[6]) const;

 protected:
  std::pair<size_t, float> index_Remainder(float t) const;
  float stateUpdate(float traj_time_s);

  struct State {
    size_t k;
    float tRemainder_s;
    float xHat[6];          // [deltax, deltav] at last discrete update
    float xHatWithoutZ[6];  // xHat - Kz * deltaz, for interpolation
    float xHatNow[6];       // interpolated
  } state_;

 private:
  Robot &robot_;
  uint64_t tstart_us_ = 0;
  Metro update_timer_{10};
  float most_recent_torques_[4];

  static constexpr size_t TRAJ_LEN = sizeof(LQG_GAINS) / sizeof(LQG_GAINS[0]);
};

std::pair<size_t, float> StateEstimatorKf::index_Remainder(float t) const {
  size_t index = static_cast<int>(t / ESTIMATOR_DT);
  if (index >= (TRAJ_LEN)) {
    return {TRAJ_LEN - 1, ESTIMATOR_DT};
  }
  float remainder = t - index * ESTIMATOR_DT;
  if ((remainder < 0) && (remainder > -1e-3)) remainder = 0;
  return {index, remainder};
}

float StateEstimatorKf::stateUpdate(float traj_time_s) {
  //
  auto index_remainder = index_Remainder(traj_time_s);
  uint64_t k = index_remainder.first;
  float tRemainder_s = index_remainder.second;
  const LqgGains &gains = LQG_GAINS[(k == 0) ? 1 : k];

  if ((state_.k == 0) && update_timer_.check()) {
    state_.k = -1;
  }

  if ((k != state_.k) && (k != (state_.k + 1))) {
    // ERROR
    SerialD.printf("Estimator Error:  k = %d, state_.k = %d\n", k, state_.k);
    std::fill(std::begin(state_.xHat), std::end(state_.xHat), 0);
    return -1;
  }

  if (k == (state_.k + 1)) {  // Kalman Filter "predict"
    // Calculate du
    static float delta_u[4];
    for (int i = 0; i < 4; ++i) {
      delta_u[i] = most_recent_torques_[i] - gains.uff[i];
    }

    // xHat = gains.Kx * xHat + gains.Ku * delta_u + gains.Kz * delta_z + k
    // xHatWithoutZ = gains.Kx * xHat + gains.Ku * delta_u + k
    static float tmpX[6], tmpU[6];
    matmul(gains.Kx, state_.xHat, tmpX);
    matmul(gains.Ku, delta_u, tmpU);
    matadd(tmpX, tmpU, state_.xHatWithoutZ);
    matadd(state_.xHatWithoutZ, gains.k, state_.xHatWithoutZ);
  }

  // Kalman Filter "update"
  {
    static float delta_z[8];
    for (int i = 0; i < 4; ++i) {
      delta_z[i] = -robot_.len(i) - gains.lff[i];
      delta_z[i + 4] = -robot_.lenDot(i) - gains.ldotff[i];
    }
    static float tmpZ[6];
    matmul(gains.Kz, delta_z, tmpZ);
    matadd(state_.xHatWithoutZ, tmpZ, state_.xHatNow);
    // safety
    clamp(&state_.xHatNow[0], -0.57, 0.57);
    clamp(&state_.xHatNow[3], -1.0, 1.0);
    // state_.xHatNow[0] = 0;
    // state_.xHatNow[3] = 0;
    state_.tRemainder_s = tRemainder_s;
  }

  // Save xHat if we did "predict"
  if (k == (state_.k + 1)) {
    state_.k = k;
    std::copy(std::begin(state_.xHatNow), std::end(state_.xHatNow),
              std::begin(state_.xHat));
    SerialD.printf("\t\t\txHat: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n",
                   traj_time_s, state_.xHat[0], state_.xHat[1], state_.xHat[2],
                   state_.xHat[3], state_.xHat[4], state_.xHat[5]);
  }

  return tRemainder_s;
}

std::pair<float, float> StateEstimatorKf::posEst(uint64_t time_us) const {
  return {state_.xHatNow[1] + LQG_GAINS[state_.k].xff[1],
          state_.xHatNow[2] + LQG_GAINS[state_.k].xff[2]};
};
std::pair<float, float> StateEstimatorKf::velEst(uint64_t time_us) const {
  return {state_.xHatNow[4] + LQG_GAINS[state_.k].vff[1],
          state_.xHatNow[5] + LQG_GAINS[state_.k].vff[2]};
};
bool StateEstimatorKf::stateEst(float traj_time_s,
                                float (&deltaXHat)[6]) const {
  float tRemainder_s = stateUpdate(traj_time_s);
  if (tRemainder_s < 0) {
    SerialD.printf("state estimation fault: %.4f\n", tRemainder_s);
    return false;
  }

  std::copy(std::begin(state_.xHatNow), std::end(state_.xHatNow),
            std::begin(deltaXHat));
  return true;
}
