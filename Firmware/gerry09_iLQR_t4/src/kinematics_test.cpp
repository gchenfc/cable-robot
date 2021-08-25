#ifndef ARDUINO

#define DEBUG_CONSTANTS
float kZeros[4] = {40.24, 43.93, 33.59, 10.96};
static constexpr float kR = 1;
static constexpr float kWidth = 4, kHeight = 3;
static constexpr float kTau = 2 * 3.1415926535897932384626433832795;
// clang-off
#include "arduino_test_utils.h"
// clang-on
// clang-off
#include "kinematics.h"
// clang-on

#include "CppUnitLite/TestHarness.h"

#include <iostream>

#include "constants.h"
#include "robot.h"
#include "utils.h"

void moveToPos(const float xy[2], Robot *robot) {
  float points[4][2] = {{kWidth, 0}, {kWidth, kHeight}, {0, kHeight}, {0, 0}};
  for (int i = 0; i < 4; ++i) {
    float dx = xy[0] - points[i][0];
    float dy = xy[1] - points[i][1];
    float l = -std::sqrt(dx * dx + dy * dy);
    robot->winches[i].setTheta(robot->winches[i].theta(l));
  }
}

TEST(Kinematics, test1) {
  Robot robot;
  Kinematics kinematics(robot);

  // simple test case for jacobian
  float xy[2] = {1, 2};
  moveToPos(xy, &robot);
  EXPECT_DOUBLES_EQUAL(-std::sqrt(13), robot.len(0), 1e-5);  // sanity checks
  EXPECT_DOUBLES_EQUAL(-std::sqrt(10), robot.len(1), 1e-5);
  EXPECT_DOUBLES_EQUAL(-std::sqrt(2), robot.len(2), 1e-5);
  EXPECT_DOUBLES_EQUAL(-std::sqrt(5), robot.len(3), 1e-5);
  // jacobian
  double expected[4][2] = {{3, -2}, {3, 1}, {-1, 1}, {-1, -2}};
  for (int i = 0; i < 4; ++i) {
    double &Fx = expected[i][0];
    double &Fy = expected[i][1];
    double norm = std::sqrt(Fx * Fx + Fy * Fy);
    Fx = Fx / norm;
    Fy = Fy / norm;
  }
  float actual[4][2];
  kinematics.jacobian(actual);
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 2; ++col) {
      EXPECT_DOUBLES_EQUAL(expected[row][col], actual[row][col], 1e-5);
    }
  }

  // all other tests:
  float x_exp = 1.1, y_exp = 2.1;
  float exp[2] = {x_exp, y_exp};
  {
    moveToPos(exp, &robot);
    // IK
    float lens[4];
    kinematics.IK(x_exp, y_exp, lens);
    for (int i = 0; i < 4; ++i) {
      EXPECT_DOUBLES_EQUAL(robot.len(i), lens[i], 1e-5);
    }

    // FK
    float x, y;
    kinematics.FK(&x, &y);
    EXPECT_DOUBLES_EQUAL(x_exp, x, 1e-6);
    EXPECT_DOUBLES_EQUAL(y_exp, y, 1e-6);
    robot.lens(lens);
    kinematics.FK(lens, &x, &y);
    EXPECT_DOUBLES_EQUAL(x_exp, x, 1e-6);
    EXPECT_DOUBLES_EQUAL(y_exp, y, 1e-6);

    // FKv
    float lDots[4];
    float v_exp[2][1] = {{1}, {-1}};
    float W[4][2];
    kinematics.jacobian(W);
    float lDotsTmp[4][1];
    matmul<4, 2, 1>(W, v_exp, lDotsTmp);
    for (int i = 0; i < 4; ++i) {
      lDots[i] = lDotsTmp[i][0];
    }
    float v_act[2];
    kinematics.FKv(lDots, W, &v_act[0], &v_act[1]);
    EXPECT_DOUBLES_EQUAL(v_exp[0][0], v_act[0], 1e-6);
    EXPECT_DOUBLES_EQUAL(v_exp[1][0], v_act[1], 1e-6);
    for (int i = 0; i < 4; ++i) {
      robot.winches[i].setThetaDot(robot.winches[i].thetaDot(lDots[i]));
    }
    kinematics.FKv(W, &v_act[0], &v_act[1]);
    EXPECT_DOUBLES_EQUAL(v_exp[0][0], v_act[0], 1e-6);
    EXPECT_DOUBLES_EQUAL(v_exp[1][0], v_act[1], 1e-6);
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
