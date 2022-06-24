#pragma once

#include "controller_interface.h"
#include "controller_gouttefarde_tracking.h"
#include "controller_ilqr.h"

/**
 * ControllerSwitchable enables dynamically switching between multiple
 * controllers.
 */
class ControllerSwitchable : public ControllerInterface {
 public:
  ControllerSwitchable(const StateEstimatorInterface* state_estimator,
                       const Robot& robot, Spray& spray)
      : controller_tracking_(state_estimator, robot),
        controller_trajectory_(state_estimator, spray),
        controller_active_(&controller_tracking_) {}

  // Common API
  void setup() override {
    controller_tracking_.setup();
    controller_trajectory_.setup();
  };
  void update() override {
    controller_tracking_.update();
    controller_trajectory_.update();
  };

  bool readSerial(AsciiParser parser, Stream& serialOut) override {
    AsciiParser parser_ = parser;
    if (parser_.checkChar('g') && parser_.checkChar('s')) {
      UNWRAP_PARSE_CHECK(uint8_t num, parser_.parseInt('\n', &num));
      switch (num) {
        case 0:
          controller_active_->hold();
          controller_active_ = &controller_tracking_;
          serialOut.println("Switched to tracking controller");
          return true;
        case 1:
          controller_active_->hold();
          controller_active_ = &controller_trajectory_;
          serialOut.println("Switched to trajectory controller");
          return true;
      }
      return false;
    }

    return controller_active_->readSerial(parser, serialOut);
  }

  /****************************** Controller API ******************************/

  // Returns true if a CAN message was sent, false otherwise (to know whether or
  // not caller should service the watchdog)
  bool encoderMsgCallback(Odrive* odrive, uint8_t winchnum) const override {
    return controller_active_->encoderMsgCallback(odrive, winchnum);
  }

  // State transition requests
  bool setupFor(ControllerState state) override {
    return controller_active_->setupFor(state);
  }
  bool goToStartTraj() override { return controller_active_->goToStartTraj(); }
  bool startTraj() override { return controller_active_->startTraj(); }
  bool stopTraj() override { return controller_active_->stopTraj(); }
  bool resetTraj() override { return controller_active_->resetTraj(); }
  bool setToTrajIndex(uint64_t index) override {
    return controller_active_->setToTrajIndex(index);
  }
  bool hold() override { return controller_active_->hold(); }
  bool release() override { return controller_active_->release(); }

  /******************************* Data Logging *******************************/
  Vector2 setpointPos() const override {
    return controller_active_->setpointPos();
  }
  float setpointTheta() const override {
    return controller_active_->setpointTheta();
  }
  Vector2 setpointVel() const override {
    return controller_active_->setpointVel();
  }
  float setpointThetaDot() const override {
    return controller_active_->setpointThetaDot();
  }

 protected:
  ControllerGouttefardeTracking controller_tracking_;
  ControllerIlqr controller_trajectory_;
  ControllerInterface* controller_active_;
};
