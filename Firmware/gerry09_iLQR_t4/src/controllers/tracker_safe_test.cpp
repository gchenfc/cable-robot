#ifndef ARDUINO

#include "../unit_test_simulator/odrive_dummy.h"
#include "../unit_test_simulator/state_estimator_dummy.h"
#include "../robot.h"
#include "setpoint_pure_pursuit.h"
#include "tracker_safe.h"

#include "../../CppUnitLite/TestHarness.h"

class TrackerTest : public TrackerSafe {
 public:
  using TrackerSafe::TrackerSafe;
  static void print_name(Stream& serialOut) {
    serialOut.print("TrackerTest");
  }
  float calcTension_N(uint8_t) override { return 0; };
};

Robot robot;
StateEstimatorDummy state_estimator;
Odrive odrive;
SetpointPurePursuit setpoint(&state_estimator);

TEST(TrackerSafe, compile_checker) {
  TrackerTest tracker(robot, &setpoint, &state_estimator);

  EXPECT(true);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
