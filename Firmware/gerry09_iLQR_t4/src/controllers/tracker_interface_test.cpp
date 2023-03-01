#ifndef ARDUINO

#include "../unit_test_simulator/arduino_test_utils.h"
#include "../unit_test_simulator/odrive_dummy.h"
#include "tracker_interface.h"

#include "../../CppUnitLite/TestHarness.h"

Odrive odrive;

class TrackerBasic : public TrackerInterface {
  using TrackerInterface::TrackerInterface;
  float calcTension_N(uint8_t) override { return 0; }
};

TEST(TrackerBasic, compile_checker) { EXPECT(true); }

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
