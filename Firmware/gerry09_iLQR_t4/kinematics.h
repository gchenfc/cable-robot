#pragma once

#include "constants.h"
#include "robot.h"
#include "utils.h"

class Kinematics {
 public:
  Kinematics(const Robot &robot) : robot_(robot) {}

  // Calculate the transpose of the "jacobian" aka "wrench matrix" in the
  // current configuration.
  // f = W'.t, where f is x/y force and t is 4-d cable tension vector
  static void jacobian(const float x, const float y, float W[4][2]);

  // Inverse and forward kinematics
  static void IK(const float x, const float y, float lengths[4]);
  static void FK(const float lengths[4], float *x, float *y);

  // Forward kinematics of velocity
  static void FKv(const float lenDots[4], const float W[4][2], float *vx,
                  float *vy);

  // Non-static versions
  void jacobian(float W[4][2]) const {
    float x, y;
    FK(&x, &y);
    jacobian(x, y, W);
  }
  void FK(float *x, float *y) const {
    float lengths[4];
    robot_.lens(lengths);
    FK(lengths, x, y);
  }
  void FKv(const float W[4][2], float *vx, float *vy) const {
    float lDots[4];
    robot_.lenDots(lDots);
    return FKv(lDots, W, vx, vy);
  }

  // Force distribution solvers
  // static void forceSolverPott(float tensions[4], float Fx, float Fy, float x,
  //                             float y);
  // static void forceSolver(float tensions[4], float Fx, float Fy, float x,
  //                         float y);
  // void forceSolver(float tensions[4], float Fx, float Fy) const {
  //   float x, y;
  //   FK(x, y);
  //   forceSolver(tensions, Fx, Fy, x, y);
  // }

 protected:
  static constexpr float kMountPoints[4][2] = {
      {kWidth, 0}, {kWidth, kHeight}, {0, kHeight}, {0, 0}};
  const Robot &robot_;
};

// impl
void Kinematics::jacobian(const float x, const float y, float W[4][2]) {
  for (uint8_t i = 0; i < 4; ++i) {
    float dx = kMountPoints[i][0] - x;
    float dy = kMountPoints[i][1] - y;
    float norm = sqrt(dx * dx + dy * dy);
    W[i][0] = dx / norm;
    W[i][1] = dy / norm;
  }
}
void Kinematics::IK(float x, float y, float lengths[4]) {
  for (int i = 0; i < 4; i++) {
    float dx = x - kMountPoints[i][0];
    float dy = y - kMountPoints[i][1];
    lengths[i] = -sqrt(dx * dx + dy * dy);
  }
}
void Kinematics::FK(const float lengths[4], float *x, float *y) {
  float a = kWidth;
  float b = -lengths[3];
  float c = -lengths[0];
  float cosC = (a * a + b * b - c * c) / (2 * a * b);
  if ((cosC <= 1) && (cosC >= -1)) {
    *x = b * cosC;
    *y = b * sqrt(1 - cosC * cosC);
  } else {
    // kinematically infeasible
    *x = 0;
    *y = 0;
  }
}
void Kinematics::FKv(const float lDots[4], const float W[4][2], float *vx,
                     float *vy) {
  // least squares: v = (W'.W)^(-1).W'.qdot
  float WTqdotx = 0, WTqdoty = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    const float &qdot = lDots[i];
    WTqdotx += W[i][0] * qdot;
    WTqdoty += W[i][1] * qdot;
  }
  float WTW[2][2] = {};
  for (uint8_t i = 0; i < 4; ++i) {
    WTW[0][0] += W[i][0] * W[i][0];
    WTW[1][0] += W[i][1] * W[i][0];
    WTW[1][1] += W[i][1] * W[i][1];
  }
  WTW[0][1] = WTW[1][0];
  float WTWinv[2][2];
  inv2x2(WTW, WTWinv);
  *vx = WTWinv[0][0] * WTqdotx + WTWinv[0][1] * WTqdoty;
  *vy = WTWinv[1][0] * WTqdotx + WTWinv[1][1] * WTqdoty;
  if (isnan(*vx)) *vx = 0;
  if (isnan(*vy)) *vy = 0;
}

// void Kinematics::forceSolverPott(float tensions[4], float Fx, float Fy, float
// x,
//                                  float y) {
//   // t = tm - pinv(W)*(f + W' * tm) = tm - W.inv(W'.W).(f + W'.tm)
//   float tm[4] = {0.6 / kR, 0.6 / kR, 0.6 / kR, 0.6 / kR};
//   // f - W'.tm
//   for (int i = 0; i < 4; ++i) {
//     Fx -= W[i][0] * tm[i];
//     Fy -= W[i][1] * tm[i];
//   }

//   // inv(W'.W)'
//   float WTW[2][2] = {};
//   for (uint8_t i = 0; i < 4; ++i) {
//     WTW[0][0] += W[i][0] * W[i][0];
//     WTW[1][0] += W[i][1] * W[i][0];
//     WTW[1][1] += W[i][1] * W[i][1];
//   }
//   WTW[0][1] = WTW[1][0];
//   float WTWinv[2][2];
//   inv2x2(WTW, WTWinv);
//   // inv(W'.W) * (f + W' * tm)
//   float intermediate[2];
//   intermediate[0] = WTWinv[0][0] * Fx + WTWinv[0][1] * Fy;
//   intermediate[1] = WTWinv[1][0] * Fx + WTWinv[1][1] * Fy;
//   // tm - W.inv(W'.W).(f + W'.tm)
//   for (int i = 0; i < 4; ++i) {
//     tensions[i] = tm[i] + W[i][0] * intermediate[0] + W[i][1] *
//     intermediate[1];
//   }
// }
// void Kinematics::forceSolver(float tensions[4], float Fx, float Fy, float x,
//                              float y) {
//   float kMountPoints[4][2] = {
//       {kWidth, 0}, {kWidth, kHeight}, {0, kHeight}, {0, 0}};
//   uint8_t i;  // cable index
//   // norms will be useful later
//   float norms[4];
//   for (i = 0; i < 4; ++i) {
//     float dx = kMountPoints[i][0] - x;
//     float dy = kMountPoints[i][1] - y;
//     norms[i] = sqrt(dx * dx + dy * dy);
//   }
//   // first assume every cable will pull with at least 0.2Nm and figure out
//   how
//   // much remaining force we need
//   for (i = 0; i < 4; ++i) {
//     tensions[i] = 0.2 / kR;
//     float qdot = getLenDot(i);
//     // static friction
//     float fs;
//     if (qdot > 0) {
//       fs = -0.1 / kR;
//     } else if (qdot == 0) {
//       fs = 0;
//     } else {
//       fs = 0.1 / kR;
//     }
//     // motor inertia
//     float Tmotor = 1.0 * qddot4des[i];
//     // sum
//     float Ti = tensions[i] + fs - Tmotor;
//     Fx -= Ti * (kMountPoints[i][0] - x) / norms[i];
//     Fy -= Ti * (kMountPoints[i][1] - y) / norms[i];
//   }

//   // identify which cables to use.
//   // The desired force will lie between 2 cables (unless F is 0 in which case
//   it
//   // doesn't matter)
//   float dx = kMountPoints[3][0] - x;
//   float dy = kMountPoints[3][1] - y;
//   float cross1 = dx * Fy - dy * Fx;
//   int cablei;
//   for (cablei = 0; cablei < 4; cablei++) {
//     dx = kMountPoints[cablei][0] - x;
//     dy = kMountPoints[cablei][1] - y;
//     float cross2 = dx * Fy - dy * Fx;
//     if ((cross1 > 0) && (cross2 <= 0))  // cross product switches from + to -
//       break;
//     cross1 = cross2;
//   }
//   // indices to reduce if-trees
//   uint8_t cable2 = cablei;            // CW side of force
//   uint8_t cable1 = (cablei + 3) % 4;  // CCW side of force
//   uint8_t notcable1 = (cablei + 1) % 4;
//   uint8_t notcable2 = (cablei + 2) % 4;
//   // solve for:
//   // (m1-x)*f1/n1 + (m2-x)*f2/n2 = F
//   //    where m1/m2 are mounting points, n1/n2 are normalizing norms, and
//   f1/f2
//   //    are cable tensions
//   // [(m1x-x)/n1  (m2x-x)/n2].[f1] = [Fx]
//   // [(m1y-y)/n1  (m2y-y)/n2] [f2]   [Fy]
//   float DX[2][2] = {{(kMountPoints[cable1][0] - x) / norms[cable1],
//                      (kMountPoints[cable2][0] - x) / norms[cable2]},
//                     {(kMountPoints[cable1][1] - y) / norms[cable1],
//                      (kMountPoints[cable2][1] - y) / norms[cable2]}};
//   float DXinv[2][2];
//   inv2x2(DX, DXinv);
//   // solve
//   tensions[cable1] += DXinv[0][0] * Fx + DXinv[0][1] * Fy;
//   tensions[cable2] += DXinv[1][0] * Fx + DXinv[1][1] * Fy;
// }
