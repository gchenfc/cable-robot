#ifndef ARDUINO

#include "../unit_test_simulator/odrive_dummy.h"
#include "../unit_test_simulator/state_estimator_dummy.h"
#include "../robot.h"
#include "setpoint_pure_pursuit.h"
#include "tracker_gouttefarde.h"

#include "../../CppUnitLite/TestHarness.h"

Robot robot;
StateEstimatorDummy state_estimator;
Odrive odrive;
SetpointPurePursuit setpoint(&state_estimator);

TEST(TrackerGouttefarde, compile_checker) {
  TrackerGouttefarde tracker(robot, odrive, &setpoint, &state_estimator);
  EXPECT(true);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
