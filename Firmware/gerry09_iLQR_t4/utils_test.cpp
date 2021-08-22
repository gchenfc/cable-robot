#ifndef ARDUINO
// clang-off
#include "arduino_test_utils.h"
// clang-on

#include "CppUnitLite/TestHarness.h"

#include "utils.h"

TEST(utils, clamp) {
  float x = 10;

  clamp(&x, 5, 15);
  EXPECT_DOUBLES_EQUAL(10, x, 1e-12);
  clamp(&x, 15, 16);
  EXPECT_DOUBLES_EQUAL(15, x, 1e-12);
  clamp(&x, 5, 17);
  EXPECT_DOUBLES_EQUAL(15, x, 1e-12);
  clamp(&x, 5, 7);
  EXPECT_DOUBLES_EQUAL(7, x, 1e-12);
}

TEST(utils, lpf) {
  float x = 0;

  lpf(&x, 1, 0.1);
  EXPECT_DOUBLES_EQUAL(0.1, x, 1e-6);
  lpf(&x, 1, 0.1);
  EXPECT_DOUBLES_EQUAL(0.19, x, 1e-6);
  lpf(&x, 1, 1);
  EXPECT_DOUBLES_EQUAL(1, x, 1e-6);
  lpf(&x, 2, 0.6);
  EXPECT_DOUBLES_EQUAL(1.6, x, 1e-6);
}

bool check_inv2x2(float A[2][2], double tol = 1e-12) {
  float B[2][2], C[2][2];
  inv2x2(A, B);
  matmul<2, 2, 2>(A, B, C);
  return (std::abs(1 - C[0][0]) < tol) && (std::abs(0 - C[0][1]) < tol) &&
         (std::abs(0 - C[1][0]) < tol) && (std::abs(1 - C[1][1]) < tol);
}

TEST(utils, inv2x2) {
  float A[2][2] = {{1, 2}, {3, 4}};
  EXPECT(check_inv2x2(A));
  float B[2][2] = {{-1, 2}, {-3, 4}};
  EXPECT(check_inv2x2(B));
  float C[2][2] = {{5, -2}, {3.2, 1.8}};
  EXPECT(check_inv2x2(C, 1e-7));
}

TEST(utils, matmul) {
  float A[2][3] = {{1, 2, 3},  //
                   {4, 5, 6}};
  float B[3][1] = {{1},  //
                   {-2},
                   {3}};
  float C[2][1];
  matmul<2, 3, 1>(A, B, C);
  EXPECT_DOUBLES_EQUAL(6, C[0][0], 1e-12);
  EXPECT_DOUBLES_EQUAL(12, C[1][0], 1e-12);
}

TEST(utils, towards) {
  float nx, ny;
  towards(1, 0, 0, 1, 1, &nx, &ny);
  EXPECT_DOUBLES_EQUAL(0.7071067812, nx, 1e-6);
  EXPECT_DOUBLES_EQUAL(0.7071067812, ny, 1e-6);
  towards(1, 1, 3, 1.5, 3.5, &nx, &ny);
  EXPECT_DOUBLES_EQUAL(1.5, nx, 1e-6);
  EXPECT_DOUBLES_EQUAL(3.5, ny, 1e-6);
  towards(std::sqrt(10), -1.1, 72.1, 0.9, 66.1, &nx, &ny);
  EXPECT_DOUBLES_EQUAL(-0.1, nx, 1e-6);
  EXPECT_DOUBLES_EQUAL(69.1, ny, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
