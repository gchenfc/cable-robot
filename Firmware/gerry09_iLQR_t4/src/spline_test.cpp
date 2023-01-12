#ifndef ARDUINO
// clang-off
#include "../unit_test_simulator/arduino_test_utils.h"
// clang-on

#include "CppUnitLite/TestHarness.h"

#include "spline.h"

#define EXPECT_XVA_EQUAL(T, tol, X_EXP, Y_EXP, VX_EXP, VY_EXP, AX_EXP, AY_EXP) \
  {                                                                            \
    std::array<float, 2> x_exp = {X_EXP, Y_EXP};                               \
    std::array<float, 2> x_act = spline.eval(T);                               \
    std::array<float, 2> v_exp = {VX_EXP, VY_EXP};                             \
    std::array<float, 2> v_act = spline.evald(T);                              \
    std::array<float, 2> a_exp = {AX_EXP, AY_EXP};                             \
    std::array<float, 2> a_act = spline.evaldd(T);                             \
    EXPECT_DOUBLES_EQUAL(std::get<0>(x_exp), std::get<0>(x_act), tol)          \
    EXPECT_DOUBLES_EQUAL(std::get<1>(x_exp), std::get<1>(x_act), tol)          \
    EXPECT_DOUBLES_EQUAL(std::get<0>(v_exp), std::get<0>(v_act), tol)          \
    EXPECT_DOUBLES_EQUAL(std::get<1>(v_exp), std::get<1>(v_act), tol)          \
    EXPECT_DOUBLES_EQUAL(std::get<0>(a_exp), std::get<0>(a_act), tol)          \
    EXPECT_DOUBLES_EQUAL(std::get<1>(a_exp), std::get<1>(a_act), tol)          \
  }

TEST(spline, polynomial) {
  // Test the polynomial utils
  PolyCoeffs1d<4> c4{1, 2, 3, 4, 5};
  PolyCoeffs1d<3> c3{1, 2, 3, 4};
  PolyCoeffs1d<0> c0{3};

  EXPECT_DOUBLES_EQUAL(5.0, polyval<4>(0.0, c4), 1e-16);
  EXPECT_DOUBLES_EQUAL(15.0, polyval<4>(1.0, c4), 1e-16);
  EXPECT_DOUBLES_EQUAL(3.0, polyval<0>(1.0, c0), 1e-16);

  // x^3 + 2x^2 + 3x + 4  ->  3x^2 + 4x + 3
  EXPECT_DOUBLES_EQUAL(3.0, std::get<2>(polyder<3>(c3)), 1e-16);
  EXPECT_DOUBLES_EQUAL(4.0, std::get<1>(polyder<3>(c3)), 1e-16);
  EXPECT_DOUBLES_EQUAL(3.0, std::get<0>(polyder<3>(c3)), 1e-16);

  // 3 -> 0
  static_assert(
      std::is_same<decltype(polyder<0>(c0)), std::array<float, 1>>::value,
      "Derivative of 0th degree poly should be 0th degree");
  EXPECT_DOUBLES_EQUAL(0.0, std::get<0>(polyder<0>(c0)), 1e-16);
}

TEST(spline, Spline) {
  // Test the PPoly class
  // Some python code was used to generate these tests.
  // See gerry05_toppra.ipynb in the art_skills repo (70ee034cc8400c4ae1b9d0bdb)
  PPoly<100> spline;

  // Test initialize to safe config
  EXPECT_DOUBLES_EQUAL(0, spline.duration(), 0);
  EXPECT_XVA_EQUAL(0.0, 0.0,    // t, tol
                   1.5, 1.5,    // x
                   0, 0,        // v
                   0, 0);       // a
  EXPECT_XVA_EQUAL(555.0, 0.0,  // t, tol
                   1.5, 1.5,    // x
                   0, 0,        // v
                   0, 0);       // a

  // Test single-segment config
  spline.add_segment(0.1, {{{{1, 2, 3, 4}}, {{5, 6, 7, 8}}}});
  EXPECT_DOUBLES_EQUAL(0.1, spline.duration(), 1e-5);
  EXPECT_DOUBLES_EQUAL(1.0, spline.get_coeffs(0).at(0).at(0), 1e-5);
  EXPECT_DOUBLES_EQUAL(2.0, spline.get_coeffs(0).at(0).at(1), 1e-5);
  EXPECT_DOUBLES_EQUAL(3.0, spline.get_coeffs(0).at(0).at(2), 1e-5);
  EXPECT_DOUBLES_EQUAL(4.0, spline.get_coeffs(0).at(0).at(3), 1e-5);
  EXPECT_DOUBLES_EQUAL(5.0, spline.get_coeffs(0).at(1).at(0), 1e-5);
  EXPECT_DOUBLES_EQUAL(6.0, spline.get_coeffs(0).at(1).at(1), 1e-5);
  EXPECT_DOUBLES_EQUAL(7.0, spline.get_coeffs(0).at(1).at(2), 1e-5);
  EXPECT_DOUBLES_EQUAL(8.0, spline.get_coeffs(0).at(1).at(3), 1e-5);
  EXPECT_DOUBLES_EQUAL(8.0, spline.get_coeffs(0).at(1).at(3), 1e-5);
  EXPECT_XVA_EQUAL(0.05, 1e-6,            // t, tol
                   4.155125, 8.365625,    // x
                   3.207500, 7.637500,    // v
                   4.300000, 13.500000);  // a
  EXPECT_XVA_EQUAL(0.09999999, 1e-6,      // t, tol
                   4.321000, 8.765000,    // x
                   3.430000, 8.350000,    // v
                   4.600000, 15.000000);  // a

  // Test 2 segments
  spline.add_segment(0.3, {{{{1, 2, 3, 4}}, {{5, 6, 7, 8}}}});
  EXPECT_XVA_EQUAL(0.05, 1e-6,            // t, tol
                   4.155125, 8.365625,    // x
                   3.207500, 7.637500,    // v
                   4.300000, 13.500000);  // a
  EXPECT_XVA_EQUAL(0.09999999, 1e-6,      // t, tol
                   4.321000, 8.765000,    // x
                   3.430000, 8.350000,    // v
                   4.600000, 15.000000);  // a
  EXPECT_XVA_EQUAL(0.15, 1e-6,            // t, tol
                   4.155125, 8.365625,    // x
                   3.207500, 7.637500,    // v
                   4.300000, 13.500000);  // a
  EXPECT_XVA_EQUAL(0.2, 1e-6,             // t, tol
                   4.321000, 8.765000,    // x
                   3.430000, 8.350000,    // v
                   4.600000, 15.000000);  // a

  // Test reset and realistic example
  spline.reset();
  spline.add_segment(0.009803361344537943,
                     {{{{-518.289381, 5.080978, 0.000000, 0.521429}},
                       {{1261.529542, -1.749690, 0.000000, -0.187755}}}});
  spline.add_segment(0.021008435681965202,
                     {{{{578.314546, -10.161956, -0.049811, 0.521429}},
                       {{-2152.101674, 35.352000, 0.329416, -0.186735}}}});
  spline.add_segment(0.03386929541047199,
                     {{{{-840.105840, 9.278216, -0.059713, 0.520408}},
                       {{1475.419016, -36.991378, 0.311046, -0.181633}}}});
  spline.add_segment(0.04813707175177775,
                     {{{{682.343536, -23.135234, -0.237926, 0.519388}},
                       {{-793.512742, 19.934093, 0.091675, -0.180612}}}});
  EXPECT_XVA_EQUAL(0.005, 1e-5,                 // t, tol
                   0.521491, -0.187641,         // x
                   0.011938, 0.077118,          // v
                   -5.386725, 34.346506);       // a
  EXPECT_XVA_EQUAL(0.01, 1e-5,                  // t, tol
                   0.521418, -0.186669,         // x
                   -0.053740, 0.343069,         // v
                   -19.641598, 68.164881);      // a
  EXPECT_XVA_EQUAL(0.015, 1e-5,                 // t, tol
                   0.520976, -0.184370,         // x
                   -0.108574, 0.522486,         // v
                   -2.292162, 3.601831);        // a
  EXPECT_XVA_EQUAL(0.021008435681965202, 1e-5,  // t, tol
                   0.520408, -0.181633,         // x
                   -0.059713, 0.311046,         // v
                   18.556433, -73.982756);      // a
  EXPECT_XVA_EQUAL(0.035, 1e-5,                 // t, tol
                   0.519090, -0.180484,         // x
                   -0.287627, 0.133711,         // v
                   -41.641294, 34.484815);      // a
  EXPECT_XVA_EQUAL(0.045, 1e-5,                 // t, tol
                   0.514814, -0.178216,         // x
                   -0.499337, 0.240505,         // v
                   -0.700682, -13.125949);      // a
  EXPECT_XVA_EQUAL(0.05, 1e-5,                  // t, tol
                   0.513265, -0.177551,         // x
                   0.0, 0.0,                    // v
                   0.0, 0.0);                   // a
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
