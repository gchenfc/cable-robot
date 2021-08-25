#pragma once

#include <Metro.h>

#include "controller_interface.h"
#include "../state_estimators/state_estimator_interface.h"

extern Odrive odrive;

class ControllerSimple : public ControllerInterface {
 public:
  ControllerSimple(const StateEstimatorInterface* state_estimator)
      : state_estimator_(state_estimator) {}

  void setup() override;
  void update() override;

  bool encoderMsgCallback(Odrive* odrive, uint8_t winchnum) const override;

  // State transition requests
  bool startTraj() override;
  bool stopTraj() override;
  bool resetTraj() override;
  bool setToTrajIndex(uint64_t index) override;
  bool hold() override;
  bool release() override;

 private:
  Metro updateTimer{50};
  const StateEstimatorInterface* state_estimator_;
  uint64_t tstart_us_;
};
