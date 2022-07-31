#ifndef ARDUINO
// clang-off
#include "../unit_test_simulator/arduino_test_utils.h"
// clang-on

#include "CppUnitLite/TestHarness.h"

#include "math.h"

bool check_inv2x2(Matrix<2, 2> A, double tol = 1e-12) {
  auto B = A.inv();
  auto C = A * B;
  return (std::abs(1 - C(0, 0)) < tol) && (std::abs(0 - C(0, 1)) < tol) &&
         (std::abs(0 - C(1, 0)) < tol) && (std::abs(1 - C(1, 1)) < tol);
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
  Matrix<2, 3> A({{1, 2, 3},  //
                  {4, 5, 6}});
  Matrix<3, 1> B({{1},  //
                  {-2},
                  {3}});
  auto C = A * B;
  EXPECT_DOUBLES_EQUAL(6, C(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(12, C(1), 1e-12);
  // alternate data access
  EXPECT_DOUBLES_EQUAL(6, C(0, 0), 1e-12);
  EXPECT_DOUBLES_EQUAL(12, C(1, 0), 1e-12);
}

TEST(utils, mat_add_sub_elementwise) {
  Matrix<2, 3> A({{1, 2, 3},  //
                  {4, 5, 6}});
  Matrix<2, 3> B({{7, 10, 9},  //
                  {10, 11, 13}});
  {  // add
    auto C = A + B;
    auto C2 = A;
    C2 += B;
    float C_expected[2][3] = {{8, 12, 12},  //
                              {14, 16, 19}};
    for (int ri = 0; ri < 2; ++ri) {
      for (int ci = 0; ci < 3; ++ci) {
        EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C(ri, ci), 1e-12);
        EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C2(ri, ci), 1e-12);
      }
    }
  }
  {  // sub
    auto C = A - B;
    auto C2 = A;
    C2 -= B;
    float C_expected[2][3] = {{-6, -8, -6},  //
                              {-6, -6, -7}};
    for (int ri = 0; ri < 2; ++ri) {
      for (int ci = 0; ci < 3; ++ci) {
        EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C(ri, ci), 1e-12);
        EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C2(ri, ci), 1e-12);
      }
    }
  }
  {  // elementwise mult/div
    auto C = A.elementwise() * B;
    auto C2 = A.elementwise() / B;
    float C_expected[2][3] = {{7, 20, 27},  //
                              {40, 55, 78}};
    float C_expected2[2][3] = {{1. / 7, 0.2, 1. / 3},  //
                               {0.4, 5. / 11, 6. / 13}};
    for (int ri = 0; ri < 2; ++ri) {
      for (int ci = 0; ci < 3; ++ci) {
        EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C(ri, ci), 1e-12);
        EXPECT_DOUBLES_EQUAL(C_expected2[ri][ci], C2(ri, ci), 1e-12);
      }
    }
  }
}

TEST(utils, mat_scalar) {
  Matrix<2, 3> A({{1, 2, 3},  //
                  {4, 5, 6}});
  auto C = A * 2;
  auto C2 = A / 0.5;
  auto C3 = A;
  C3 *= 2;
  auto C4 = A;
  C4 /= 0.5;
  auto C5 = 2 * A;
  float C_expected[2][3] = {{2, 4, 6},  //
                            {8, 10, 12}};
  for (int ri = 0; ri < 2; ++ri) {
    for (int ci = 0; ci < 3; ++ci) {
      EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C(ri, ci), 1e-12);
      EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C2(ri, ci), 1e-12);
      EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C3(ri, ci), 1e-12);
      EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C4(ri, ci), 1e-12);
      EXPECT_DOUBLES_EQUAL(C_expected[ri][ci], C5(ri, ci), 1e-12);
    }
  }
}

TEST(utils, transpose) {
  Matrix<2, 3> A({{1, 2, 3},  //
                  {4, 5, 6}});
  auto AT = A.T();
  float AT_expected[3][2] = {{1, 4},  //
                             {2, 5},
                             {3, 6}};
  for (int ri = 0; ri < 3; ++ri) {
    for (int ci = 0; ci < 2; ++ci) {
      EXPECT_DOUBLES_EQUAL(AT_expected[ri][ci], AT(ri, ci), 1e-12);
    }
  }
}

TEST(utils, least_squares) {
  Matrix<3, 2> A({{1, 2},  //
                  {3, 4},
                  {5, 6}});
  Matrix<3, 2> b({{10, 50},  // solve 2 problems at the same time
                  {11, 51},
                  {12, 52}});
  auto x_actual = leastSquares(A, b);
  Matrix<2, 2> x_expected({{-9, -49},  // computed w/ scipy.optimize.lsq_linear
                           {9.5, 49.5}});
  for (int ri = 0; ri < 2; ++ri) {
    for (int ci = 0; ci < 2; ++ci) {
      EXPECT_DOUBLES_EQUAL(x_expected(ri, ci), x_actual(ri, ci), 1e-3);
    }
  }
}

TEST(utils, converters) {
  Matrix<2, 3> A({{1, 2, 3},  //
                  {4, 5, 6}});
  {
    float A_array[2][3];
    A.toCMatrix(A_array);
    for (int ri = 0; ri < 2; ++ri) {
      for (int ci = 0; ci < 3; ++ci) {
        EXPECT_DOUBLES_EQUAL(A(ri, ci), A_array[ri][ci], 1e-12);
      }
    }
    Vector<2> col = A.col(1);
    for (int ri = 0; ri < 2; ++ri) {
      EXPECT_DOUBLES_EQUAL(A(ri, 1), col(ri), 1e-12);
    }
    Vector<3> row = A.row(1);
    for (int ci = 0; ci < 3; ++ci) {
      EXPECT_DOUBLES_EQUAL(A(1, ci), row(ci), 1e-12);
    }
    float B[3];
    A.row(0).toCVector(B);
    for (int ci = 0; ci < 3; ++ci) {
      EXPECT_DOUBLES_EQUAL(A(0, ci), B[ci], 1e-12);
    }
  }

  {
    float(&A_array)[2][3] = A.asCMatrix();
    A_array[0][0] = 100;
    A_array[0][1] = 101;
    A_array[0][2] = 102;
    A_array[1][0] = 103;
    A_array[1][1] = 104;
    A_array[1][2] = 105;
    EXPECT_DOUBLES_EQUAL(100, A(0, 0), 1e-12);
    EXPECT_DOUBLES_EQUAL(101, A(0, 1), 1e-12);
    EXPECT_DOUBLES_EQUAL(102, A(0, 2), 1e-12);
    EXPECT_DOUBLES_EQUAL(103, A(1, 0), 1e-12);
    EXPECT_DOUBLES_EQUAL(104, A(1, 1), 1e-12);
    EXPECT_DOUBLES_EQUAL(105, A(1, 2), 1e-12);
    Vector<3> row = A.row(0);
    float(&row_vec)[3] = row.asCVector();
    row_vec[0] = 30;
    row_vec[1] = 31;
    row_vec[2] = 32;
    EXPECT_DOUBLES_EQUAL(30, row(0), 1e-12);
    EXPECT_DOUBLES_EQUAL(31, row(1), 1e-12);
    EXPECT_DOUBLES_EQUAL(32, row(2), 1e-12);
  }
}

TEST(utils, vecadd) {
  Vector<3> A({1, 2, 3});
  Vector<3> B({7, 8, 9});
  Vector<3> C = A + B;
  float C_expected[3] = {8, 10, 12};
  for (int ri = 0; ri < 3; ++ri) {
    EXPECT_DOUBLES_EQUAL(C_expected[ri], C(ri), 1e-12);
  }
}

// TEST(utils, vector_stuff) {
//   const Vector<4> vec = {1, 2, 3, 4};
//   EXPECT_DOUBLES_EQUAL(30, norm2<4>(vec), 0.0);
//   EXPECT_DOUBLES_EQUAL(5.4772255751, norm<4>(vec), 1e-5);

//   float unit_vec[4];
//   memcpy(unit_vec, vec, sizeof(vec));
//   normalize<4>(unit_vec);
//   EXPECT_DOUBLES_EQUAL(0.1825741858, unit_vec[0], 1e-5);
//   EXPECT_DOUBLES_EQUAL(0.3651483717, unit_vec[1], 1e-5);
//   EXPECT_DOUBLES_EQUAL(0.5477225575, unit_vec[2], 1e-5);
//   EXPECT_DOUBLES_EQUAL(0.7302967433, unit_vec[3], 1e-5);

//   const float vec2[4] = {8, 7, 6, 5};
//   EXPECT_DOUBLES_EQUAL(8 + 14 + 18 + 20, dot<4>(vec, vec2), 0.0);
// }

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
