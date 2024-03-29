/* Unit tests for Vector.h */

#ifndef ARDUINO
// clang-off
#include "../unit_test_simulator/arduino_test_utils.h"
// clang-on

#include "CppUnitLite/TestHarness.h"

#include "Vector.h"

#define EXPECT_VECTOR3_EQUAL(x, y, z, actual, tol) \
  {                                                \
    Vector<3> act = actual;                        \
    EXPECT_DOUBLES_EQUAL(x, act[0], tol);          \
    EXPECT_DOUBLES_EQUAL(y, act[1], tol);          \
    EXPECT_DOUBLES_EQUAL(z, act[2], tol);          \
  }
#define COMMA ,

TEST(Vector, head_tail_and_stuff) {
  Vector<5> v{1, 2, 3, 4, 5};
  EXPECT_VECTOR3_EQUAL(1, 2, 3, head<3>(v), 1e-12);
  EXPECT_VECTOR3_EQUAL(3, 4, 5, tail<3>(v), 1e-12);
  EXPECT_VECTOR3_EQUAL(2, 3, 4, mid<1 COMMA 3>(v), 1e-12);
  EXPECT_VECTOR3_EQUAL(3, 0, 0, tail<3>(pad_zeros<5>(head<3>(v))), 1e-12);
  EXPECT_VECTOR3_EQUAL(3, 2, 0, pad_zeros<3>(Vector<2>{3 COMMA 2}), 1e-12);
  EXPECT_VECTOR3_EQUAL(3, 2, 1, pad_zeros<3>(Vector<3>{3 COMMA 2 COMMA 1}),
                       1e-12);
}

TEST(Vector, math) {
  Vector<3> v1{1, 2, 3};
  Vector<3> v2{4, 5, 6};

  // Test elementwise operations
  EXPECT_VECTOR3_EQUAL(5, 7, 9, v1 + v2, 1e-12);
  EXPECT_VECTOR3_EQUAL(-3, -3, -3, v1 - v2, 1e-12);
  EXPECT_VECTOR3_EQUAL(4, 10, 18, v1 * v2, 1e-12);

  // Test scalar operations
  EXPECT_VECTOR3_EQUAL(2, 4, 6, v1 * 2, 1e-12);
  EXPECT_VECTOR3_EQUAL(2, 4, 6, 2 * v1, 1e-12);
  EXPECT_VECTOR3_EQUAL(0.5, 1, 1.5, v1 / 2, 1e-12);
  EXPECT_VECTOR3_EQUAL(6, 3, 2, 6 / v1, 1e-12);

  // Test dot product
  EXPECT_DOUBLES_EQUAL(32, dot(v1, v2), 1e-12);

  // Test cross product
  EXPECT_VECTOR3_EQUAL(-3, 6, -3, cross(v1, v2), 1e-12);
  EXPECT_VECTOR3_EQUAL(3, -6, 3, cross(v2, v1), 1e-12);
  EXPECT_DOUBLES_EQUAL(-2, cross(Vector<2>{1, 2}, {3, 4}), 1e-12);

  // Test norm
  EXPECT_DOUBLES_EQUAL(14, norm2(v1), 1e-12);
  EXPECT_DOUBLES_EQUAL(std::sqrt(14), norm(v1), 1e-5);
  EXPECT_VECTOR3_EQUAL(0.2672612419, 0.5345224838, 0.8017837257, normalized(v1),
                       1e-5);

  // Test clamp
  Vector<6> min = {0, 1, 2, 3, 4, 5};
  Vector<6> max = {6, 7, 8, 9, 9, 9};
  Vector<6> x = {-1, -4, 3, 7, 10, 8.9};
  Vector<6> y = clamp(x, min, max);  // copy
  EXPECT_DOUBLES_EQUAL(0, y[0], 1e-12);
  EXPECT_DOUBLES_EQUAL(1, y[1], 1e-12);
  EXPECT_DOUBLES_EQUAL(3, y[2], 1e-12);
  EXPECT_DOUBLES_EQUAL(7, y[3], 1e-12);
  EXPECT_DOUBLES_EQUAL(9, y[4], 1e-12);
  EXPECT_DOUBLES_EQUAL(8.9, y[5], 1e-6);
  EXPECT_DOUBLES_EQUAL(-1, x[0], 1e-12);  // (don't modify x)
  clamp_(x, min, max);                    // inplace
  EXPECT_DOUBLES_EQUAL(0, x[0], 1e-12);
  EXPECT_DOUBLES_EQUAL(1, x[1], 1e-12);
  EXPECT_DOUBLES_EQUAL(3, x[2], 1e-12);
  EXPECT_DOUBLES_EQUAL(7, x[3], 1e-12);
  EXPECT_DOUBLES_EQUAL(9, x[4], 1e-12);
  EXPECT_DOUBLES_EQUAL(8.9, x[5], 1e-6);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
