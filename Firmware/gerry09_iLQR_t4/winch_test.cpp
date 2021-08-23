#ifndef ARDUINO

#include "winch.h"

#include "CppUnitLite/TestHarness.h"

TEST(Winch, test1) {
  Winch winch(1.23, 100 / 2 / M_PI);

  EXPECT_DOUBLES_EQUAL(1.23, winch.zero(), 1e-6);

  winch.setTheta(0);
  EXPECT_DOUBLES_EQUAL(-123, winch.len(), 1e-12);
  winch.setTheta(53);
  EXPECT_DOUBLES_EQUAL(5177, winch.len(), 1e-12);

  winch.setThetaDot(1);
  EXPECT_DOUBLES_EQUAL(100, winch.lenDot(), 1e-12);

  // theta() function
  EXPECT_DOUBLES_EQUAL(winch.theta(), winch.theta(winch.len()), 1e-12);
  winch.setTheta(1.234);
  EXPECT_DOUBLES_EQUAL(winch.theta(), winch.theta(winch.len()), 1e-12);
  winch.setTheta(-987);
  EXPECT_DOUBLES_EQUAL(winch.theta(), winch.theta(winch.len()), 1e-12);

  // thetaDot() function
  EXPECT_DOUBLES_EQUAL(winch.thetaDot(), winch.thetaDot(winch.lenDot()), 1e-12);
  winch.setThetaDot(1.234);
  EXPECT_DOUBLES_EQUAL(winch.thetaDot(), winch.thetaDot(winch.lenDot()), 1e-12);
  winch.setThetaDot(-987);
  EXPECT_DOUBLES_EQUAL(winch.thetaDot(), winch.thetaDot(winch.lenDot()), 1e-12);

  winch.setZero();
  EXPECT_DOUBLES_EQUAL(-987, winch.zero(), 1e-12);
}

TEST(Winch, valid) {
  manual_micros = 1e6;

  Winch winch(1.23, 100);
  EXPECT(!winch.isThetaValid());
  EXPECT(!winch.isThetaDotValid());

  winch.setTheta(5.5);
  EXPECT(winch.isThetaValid());
  EXPECT(!winch.isThetaDotValid());

  manual_micros += 1e3;
  winch.setThetaDot(8.8);
  EXPECT(winch.isThetaValid());
  EXPECT(winch.isThetaDotValid());

  manual_micros += 1e3;
  EXPECT(!winch.isThetaValid(2000));
  EXPECT(winch.isThetaValid(2001));
  EXPECT(winch.isThetaDotValid());

  manual_micros += 3e3 - 1;
  EXPECT(winch.isThetaValid());
  manual_micros += 1;
  EXPECT(!winch.isThetaValid());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
