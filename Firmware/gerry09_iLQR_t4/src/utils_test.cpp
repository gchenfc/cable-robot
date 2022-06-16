#ifndef ARDUINO
// clang-off
#include "../unit_test_simulator/arduino_test_utils.h"
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

bool check_inv2x2(float (&A)[2][2], double tol = 1e-12) {
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

TEST(utils, matadd) {
  float A[2][3] = {{1, 2, 3},  //
                   {4, 5, 6}};
  float B[2][3] = {{7, 8, 9},  //
                   {10, 11, 12}};
  float C[2][3];
  matadd<2, 3>(A, B, C);
  float C_expected[2][3] = {{8, 10, 12},  //
                            {14, 16, 18}};
  for (int ri = 0; ri < 2; ++ri) {
    for (int ci = 0; ci < 3; ++ci) {
      EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C[ri][ci], 1e-12);
    }
  }
}

TEST(utils, vecadd) {
  float A[3] = {1, 2, 3};
  float B[3] = {7, 8, 9};
  float C[3];
  matadd<3>(A, B, C);
  float C_expected[3] = {8, 10, 12};
  for (int ri = 0; ri < 3; ++ri) {
    EXPECT_DOUBLES_EQUAL(C_expected[ri], C[ri], 1e-12);
  }
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

TEST(utils, vector_stuff) {
  const float vec[4] = {1, 2, 3, 4};
  EXPECT_DOUBLES_EQUAL(30, norm2<4>(vec), 0.0);
  EXPECT_DOUBLES_EQUAL(5.4772255751, norm<4>(vec), 1e-5);

  float unit_vec[4];
  memcpy(unit_vec, vec, sizeof(vec));
  normalize<4>(unit_vec);
  EXPECT_DOUBLES_EQUAL(0.1825741858, unit_vec[0], 1e-5);
  EXPECT_DOUBLES_EQUAL(0.3651483717, unit_vec[1], 1e-5);
  EXPECT_DOUBLES_EQUAL(0.5477225575, unit_vec[2], 1e-5);
  EXPECT_DOUBLES_EQUAL(0.7302967433, unit_vec[3], 1e-5);

  const float vec2[4] = {8, 7, 6, 5};
  EXPECT_DOUBLES_EQUAL(8 + 14 + 18 + 20, dot<4>(vec, vec2), 0.0);
}

TEST(utils, Timestamped) {
  Timestamped<float> f(0);
  EXPECT_DOUBLES_EQUAL(0.0, f, 1e-5);
  EXPECT(!f.isValid());
  f = 1.23;
  EXPECT_DOUBLES_EQUAL(1.23, f, 1e-5);
  EXPECT(f.isValid());

  getMicros() = 123456L;
  f = 8.23;
  getMicros() += 1234;
  EXPECT(!f.isValid(1234));
  EXPECT(f.isValid(1235));
  EXPECT_DOUBLES_EQUAL(8.23, f, 1e-5);
  getMicros() = 123456L + 4999L;
  EXPECT(f.isValid());
  getMicros() = 123456L + 5000L;
  EXPECT(!f.isValid());
  EXPECT_DOUBLES_EQUAL(8.23, f, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
