#ifndef ARDUINO

#include "../unit_test_simulator/odrive_dummy.h"
#include "../unit_test_simulator/state_estimator_dummy.h"
#include "../unit_test_simulator/ostreams.h"
#include "setpoint_spline.h"

#include "../../CppUnitLite/TestHarness.h"

#define EXPECT_VECTOR3_EQUAL(x, y, z, actual, tol) \
  {                                                \
    Vector<3> act = actual;                        \
    EXPECT_DOUBLES_EQUAL(x, act[0], tol);          \
    EXPECT_DOUBLES_EQUAL(y, act[1], tol);          \
    EXPECT_DOUBLES_EQUAL(z, act[2], tol);          \
  }

TEST(SetpointSpline, constructor) {
  StateEstimatorDummy state_estimator;
  SetpointSpline setpoint(&state_estimator);

  EXPECT(true);  // this is mostly just a compile test
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
