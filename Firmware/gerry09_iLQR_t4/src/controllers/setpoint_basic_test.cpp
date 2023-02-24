#ifndef ARDUINO

#include "../unit_test_simulator/odrive_dummy.h"
#include "../unit_test_simulator/state_estimator_dummy.h"
#include "../unit_test_simulator/ostreams.h"
#include "setpoint_basic.h"

#include "../../CppUnitLite/TestHarness.h"

#define EXPECT_VECTOR3_EQUAL(x, y, z, actual, tol) \
  {                                                \
    Vector<3> act = actual;                        \
    EXPECT_DOUBLES_EQUAL(x, act[0], tol);          \
    EXPECT_DOUBLES_EQUAL(y, act[1], tol);          \
    EXPECT_DOUBLES_EQUAL(z, act[2], tol);          \
  }

#define PRINT_POS(x) \
  std::cout << "Checkpoint L" << __LINE__ << ", pos: " << x << std::endl;

class SetpointTest : public SetpointBasic {
 public:
  using SetpointBasic::SetpointBasic;

  X desPos(float t) override { return {t * t, t, 0}; }  // TODO: try non-0 theta
  V desVel(float t) override { return {2 * t, 1, 0}; }
  A desAcc(float) override { return {2, 0, 0}; }
  bool isDone(float) override { return false; }

  void setTravelSpeed(float speed) { travel_speed_ = speed; }
  void setLimits(const X& min, const X& max) {
    limits_min_ = min;
    limits_max_ = max;
  }
};

TEST(SetpointBasic, state_control) {
  StateEstimatorDummy state_estimator;
  state_estimator.setPos(1.5, 1.7);
  SetpointTest setpoint(&state_estimator);

  setpoint.setTravelSpeed(1.0);
  setpoint.setLimits({-10, -10, -10}, {999, 999, 999});

  auto update_time_us = [&setpoint](uint64_t time_us) {
    setpoint.update();
    set_time_us(time_us);
    setpoint.update();
  };

  // Initialize improperly: expect safe state
  update_time_us(100);
  EXPECT_VECTOR3_EQUAL(1.5, 1.7, 0, setpoint.setpointPos(), 1e-6);
  EXPECT_VECTOR3_EQUAL(0, 0, 0, setpoint.setpointVel(), 1e-6);
  EXPECT_VECTOR3_EQUAL(0, 0, 0, setpoint.setpointAcc(), 1e-6);
  update_time_us(200);
  EXPECT_VECTOR3_EQUAL(1.5, 1.7, 0, setpoint.setpointPos(), 1e-6);
  EXPECT_VECTOR3_EQUAL(0, 0, 0, setpoint.setpointVel(), 1e-6);
  EXPECT_VECTOR3_EQUAL(0, 0, 0, setpoint.setpointAcc(), 1e-6);

  // Now try out the API
  // travel()
  update_time_us(300'000);
  setpoint.travel();  // linearly interpolate from (1.5, 1.7) to start (0, 0, 0)
  EXPECT_VECTOR3_EQUAL(1.5, 1.7, 0, setpoint.setpointPos(), 1e-6);
  update_time_us(1300'000);             // after one second
  constexpr float dist = 2.2671568098;  // sqrt(1.5^2 + 1.7^2)
  EXPECT_VECTOR3_EQUAL(1.5 / dist * (dist - 1), 1.7 / dist * (dist - 1), 0,
                       setpoint.setpointPos(), 1e-6);
  update_time_us(300'000 + static_cast<uint64_t>(1000'000 * dist) -
                 1);  // almost done traveling
  EXPECT_VECTOR3_EQUAL(0, 0, 0, setpoint.setpointPos(), 1e-5);
  EXPECT_LONGS_EQUAL(SetpointBasic::State::INTERMEDIATE_TRAVEL,
                     setpoint.getState());
  state_estimator.setPos(0, 0);  // assume the controller worked
  update_time_us(300'000 + (1000'000 * dist) + 1);  // done traveling
  EXPECT_LONGS_EQUAL(SetpointBasic::State::HOLD, setpoint.getState());
  EXPECT_VECTOR3_EQUAL(0, 0, 0, setpoint.setpointPos(), 1e-6);
  update_time_us(1'000'000'000);  // hold forever, since we only did travel

  // start after travel()
  setpoint.start();
  EXPECT_LONGS_EQUAL(SetpointBasic::State::INTERMEDIATE_TRAVEL,
                     setpoint.getState());
  setpoint.update();
  EXPECT_LONGS_EQUAL(SetpointBasic::State::RUNNING, setpoint.getState());
  // PRINT_POS(setpoint.setpointPos());
  EXPECT_VECTOR3_EQUAL(0, 0, 0, setpoint.setpointPos(), 1e-6);
  update_time_us(1'000'000'000 + 1'000'000);
  EXPECT_VECTOR3_EQUAL(1.0, 1.0, 0, setpoint.setpointPos(), 1e-6);
  update_time_us(1'000'000'000 + 2'000'000);
  EXPECT_VECTOR3_EQUAL(4.0, 2.0, 0, setpoint.setpointPos(), 1e-6);

  // pause after start()
  setpoint.pause();
  EXPECT_LONGS_EQUAL(SetpointBasic::State::HOLD, setpoint.getState());
  update_time_us(1'000'000'000 + 3'000'000);
  state_estimator.setPos(4.0, 2.0);  // assume the controller worked

  // start after pause()
  setpoint.start();
  EXPECT_LONGS_EQUAL(SetpointBasic::State::INTERMEDIATE_TRAVEL,
                     setpoint.getState());
  setpoint.update();
  EXPECT_LONGS_EQUAL(SetpointBasic::State::RUNNING, setpoint.getState());
  EXPECT_VECTOR3_EQUAL(4.0, 2.0, 0, setpoint.setpointPos(), 1e-6);
  update_time_us(1'000'000'000 + 4'000'000);
  EXPECT_VECTOR3_EQUAL(9.0, 3.0, 0, setpoint.setpointPos(), 1e-6);
  state_estimator.setPos(9.0, 2.8);  // assume the controller almost worked

  // stop
  setpoint.stop();
  EXPECT_LONGS_EQUAL(SetpointBasic::State::HOLD, setpoint.getState());
  update_time_us(1'000'000'000 + 10'000'000);
  EXPECT_VECTOR3_EQUAL(9.0, 2.8, 0, setpoint.setpointPos(), 1e-6);

  // start after stop() and after move
  state_estimator.setPos(9.0, 0.0);  // assume someone manually moved the robot
  setpoint.start();
  EXPECT_LONGS_EQUAL(SetpointBasic::State::INTERMEDIATE_TRAVEL,
                     setpoint.getState());
  EXPECT_VECTOR3_EQUAL(9.0, 0, 0, setpoint.setpointPos(), 1e-6);
  update_time_us(1'000'000'000 + 10'050'000);
  EXPECT_VECTOR3_EQUAL(8.95, 0, 0, setpoint.setpointPos(), 1e-6);
  update_time_us(1'000'000'000 + 10'150'000);
  EXPECT_VECTOR3_EQUAL(8.85, 0, 0, setpoint.setpointPos(), 1e-6);
  EXPECT_LONGS_EQUAL(SetpointBasic::State::INTERMEDIATE_TRAVEL,
                     setpoint.getState());
  state_estimator.setPos(0, 0);  // assume the controller worked
  update_time_us(1'000'000'000 + 19'000'001);
  EXPECT_LONGS_EQUAL(SetpointBasic::State::RUNNING, setpoint.getState());
  EXPECT_VECTOR3_EQUAL(0, 0, 0, setpoint.setpointPos(), 1e-6);
  update_time_us(1'000'000'000 + 21'000'000);
  EXPECT_VECTOR3_EQUAL(4.0, 2.0, 0, setpoint.setpointPos(), 1e-5);
  state_estimator.setPos(4.0, 2.0);  // assume the controller worked

  // advanceTo, in the middle of running
  setpoint.update();
  setpoint.advanceTo(5.0);  // setpoint (25, 5, 0)
  setpoint.update();
  EXPECT_LONGS_EQUAL(SetpointBasic::State::INTERMEDIATE_TRAVEL,
                     setpoint.getState());
  Vector<2> dx{21., 3.};
  update_time_us(1'000'000'000 + 21'000'000 + 2'000'000);  // after 2 seconds
  EXPECT_VECTOR3_EQUAL(4.0 + dx.at(0) / norm(dx) * 2,
                       2.0 + dx.at(1) / norm(dx) * 2, 0, setpoint.setpointPos(),
                       1e-6);
  update_time_us(1'000'000'000 + 21'000'000 +
                 static_cast<uint64_t>(1'000'000UL * norm(dx)) - 1);
  EXPECT_LONGS_EQUAL(SetpointBasic::State::INTERMEDIATE_TRAVEL,
                     setpoint.getState());
  state_estimator.setPos(25, 5);  // assume the controller worked
  update_time_us(1'000'000'000UL + 21'000'000UL +
                 static_cast<uint64_t>(1'000'000UL * norm(dx)) + 1);
  EXPECT_LONGS_EQUAL(SetpointBasic::State::RUNNING, setpoint.getState());
  EXPECT_VECTOR3_EQUAL(25, 5, 0, setpoint.setpointPos(), 1e-4);
}

TEST(SetpointBasic, clamp_limits) {
  StateEstimatorDummy state_estimator;
  state_estimator.setPos(1.5, 1.7);
  SetpointTest setpoint(&state_estimator);

  setpoint.setTravelSpeed(1.0);
  setpoint.setLimits({0, 0, 0}, {1, 1, 1});

  auto update_time_us = [&setpoint](uint64_t time_us) {
    setpoint.update();
    set_time_us(time_us);
    setpoint.update();
  };

  // If we are currently at 1.5, 1.7, during travel stroke not succeed
  setpoint.initialize();
  EXPECT_LONGS_EQUAL(SetpointBasic::Status::UNINITIALIZED,
                     setpoint.getStatus());
  setpoint.travel();
  EXPECT_LONGS_EQUAL(SetpointBasic::Status::UNINITIALIZED,
                     setpoint.getStatus());
  state_estimator.setPos(0.5, 0.7);
  setpoint.initialize();
  update_time_us(10);
  setpoint.travel();
  update_time_us(10);
  EXPECT_LONGS_EQUAL(SetpointBasic::Status::NOMINAL, setpoint.getStatus());
  EXPECT_LONGS_EQUAL(SetpointBasic::State::INTERMEDIATE_TRAVEL,
                     setpoint.getState());
  EXPECT_VECTOR3_EQUAL(0.5, 0.7, 0, setpoint.setpointPos(), 1e-4);
  update_time_us(20);
  // When we finish traveling, we should stay there
  uint64_t tnow = 20;
  while (setpoint.getState() == SetpointBasic::State::INTERMEDIATE_TRAVEL) {
    auto setpos = setpoint.setpointPos();
    state_estimator.setPos(std::get<0>(setpos),
                           std::get<1>(setpos));  // assume controller works
    update_time_us(tnow += 100);
  }
  update_time_us(1'000'000'000);
  EXPECT_LONGS_EQUAL(SetpointBasic::Status::NOMINAL, setpoint.getStatus());
  EXPECT_LONGS_EQUAL(SetpointBasic::State::HOLD, setpoint.getState());
  EXPECT_VECTOR3_EQUAL(0., 0., 0, setpoint.setpointPos(), 1e-4);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#endif
