#pragma once

#include <tuple>

#include "controller_interface.h"

/**
 * ControllerSwitchable enables dynamically switching between multiple
 * controllers.
 * Usage:
 * ```c++
 * ControllerGouttefardeTracking<ControllerTracking>
 * controller1(&state_estimator, robot);
 * ControllerGouttefardeTracking<ControllerTrackingSpline>
 * controller2(&state_estimator); ControllerIlqr controller3(&state_estimator,
 * spray);
 *
 * ControllerSwitchable controller(controller1, controller2, controller3);
 *
 * main() {
 *   controller.setup();
 *   while (true) { controller.update(); }
 * }
 * ```
 */
template <typename... Controllers>
class ControllerSwitchable : public ControllerInterface {
  template <int I, typename T = void>
  using RECURSE =
      typename std::enable_if<I + 1 < sizeof...(Controllers), T>::type;
  template <int I, typename T = void>
  using TERMINATE =
      typename std::enable_if<I + 1 == sizeof...(Controllers), T>::type;

 public:
  ControllerSwitchable(Controllers&... controllers)
      : controllers_(controllers...),
        controller_active_(&std::get<0>(controllers_)) {}

  // Common API
  void setup() override { setup_<0>(); }
  void update() override { update_<0>(); }

  bool readSerial(AsciiParser parser, Stream& serialOut) override {
    AsciiParser parser_ = parser;
    if (parser_.checkChar('g') && parser_.checkChar('s')) {
      if ((!parser_.checkDone()) && (parser_.get_buffer_cur()[0] == '?')) {
        print_options(serialOut);
        return true;
      }
      UNWRAP_PARSE_CHECK(uint8_t num, parser_.parseInt('\n', &num));
      if (try_activate(num, serialOut)) return true;
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
  std::tuple<Controllers&...> controllers_;
  ControllerInterface* controller_active_;

  // Template magic
  // TODO(gerry): std::apply?
  template <int I>
  RECURSE<I, void> setup_() {
    std::get<I>(controllers_).setup();
    setup_<I + 1>();
  };
  template <int I>
  TERMINATE<I, void> setup_() {
    std::get<I>(controllers_).setup();
  };

  template <int I>
  RECURSE<I, void> update_() {
    std::get<I>(controllers_).update();
    update_<I + 1>();
  };
  template <int I>
  TERMINATE<I, void> update_() {
    std::get<I>(controllers_).update();
  };

  template <int I = 0>
  RECURSE<I, bool> try_activate(int i, Stream& serialOut) {
    return try_activate_<I>(i, serialOut) || try_activate<I + 1>(i, serialOut);
  }
  template <int I = 0>
  TERMINATE<I, bool> try_activate(int i, Stream& serialOut) {
    return try_activate_<I>(i, serialOut);
  }

  template <int I>
  bool try_activate_(int i, Stream& serialOut) {
    if (i == I) {
      controller_active_->hold();
      controller_active_ = &std::get<I>(controllers_);
      serialOut.print("Switched to controller ");
      std::get<I>(controllers_).print_name(serialOut);
      serialOut.println();
      return true;
    }
    return false;
  }

  template <int I = 0>
  RECURSE<I, void> print_options(Stream& serialOut) {
    serialOut.printf("Option %d: ", I);
    std::get<I>(controllers_).print_name(serialOut);
    serialOut.println();
    print_options<I + 1>(serialOut);
  }
  template <int I = 0>
  TERMINATE<I, void> print_options(Stream& serialOut) {
    serialOut.printf("Option %d: ", I);
    std::get<I>(controllers_).print_name(serialOut);
    serialOut.println();
  }
};
