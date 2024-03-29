/**
 * @file gerry09_iLQR_t4.ino
 * @author Gerry Chen
 * @brief The main code file for the micro-controller (Teenys 4.x) used to
 * control a cable robot.
 *
 * The high-level control-flow is as follows:
 *   - Computer (master) <-> Teensy (slave):
 *        The computer must poll for everything.  Every communication exchange
 *        is the equivalent of a remote function call on the Teensy
 *   - Teensy <-> Odrive
 *        The ODrive periodically sends pos/vel updates (thereby initiating a
 *        communication exchange).
 *          During idle state, the Teensy does nothing.
 *          During run state, the Teensy immediately sends a torque response.
 *        For state transitions, the Teensy initiates the communications to set
 *        the ODrive into the appropriate state.
 *
 * During the run state, the Teensy's `ODrive` objects are the ones that receive
 * the pos/vel updates.  They then update the `Robot` and request from the
 * `Controller` the torque command to respond with (this update should be very
 * fast and just executing a locally-linear closed-loop feedback).  The
 * `Controller` updates asynchronously to update the locally-linear feedback
 * controller according to the trajectory and/or computer commands.
 */

#include <Metro.h>

#include "src/constants.h"
#include "src/robot.h"
#include "src/state_estimators/state_estimator_first_order.h"
// #include "src/state_estimators/state_estimator_kf.h"
// #include "src/controllers/controller_simple.h"
// #include "src/controllers/controller_tracking.h"
#define OLDx
#ifdef OLD
#include "src/controllers/controller_ilqr.h"
// #include "src/controllers/controller_lqg.h"
// #include "src/controllers/controller_gouttefarde.h"
#include "src/controllers/controller_tracking.h"
#include "src/controllers/controller_tracking_spline.h"
#include "src/controllers/controller_gouttefarde_tracking.h"
#else
#include "src/controllers/setpoint_pure_pursuit.h"
#include "src/controllers/setpoint_waypoints.h"
#include "src/controllers/setpoint_spline.h"
#include "src/controllers/tracker_gouttefarde.h"
#include "src/controllers/controller_tracker_setpoint.h"
#endif
#include "src/controllers/controller_switchable.h"
#include "src/communication/odrive_can.h"
#include "src/spray.h"
#include "src/arm.h"
#include "src/estop.h"
#include "src/communication/debug.h"
#include "src/communication/slave.h"

Spray spray(btSerial);
Robot robot{};
StateEstimatorFirstOrder state_estimator(robot);
// StateEstimatorKf state_estimator(robot);
// ControllerSimple controller(&state_estimator);
// ControllerTracking controller(&state_estimator);
#ifdef OLD
ControllerIlqr controller3(&state_estimator, spray);
// ControllerLqg controller(&state_estimator);
ControllerGouttefardeTracking<ControllerTracking> controller1(&state_estimator,
                                                              robot);
ControllerGouttefardeTracking<ControllerTrackingSpline> controller2(
    &state_estimator, robot);
ControllerSwitchable<ControllerGouttefardeTracking<ControllerTracking>,
                     ControllerGouttefardeTracking<ControllerTrackingSpline>,
                     ControllerIlqr>
    controller(controller1, controller2, controller3);

#else
SetpointPurePursuit setpoint1(&state_estimator);
SetpointWaypoints setpoint2(&state_estimator);
SetpointSpline setpoint3(&state_estimator);
TrackerGouttefarde tracker1(robot, &setpoint1, &state_estimator);
TrackerGouttefarde tracker2(robot, &setpoint2, &state_estimator);
TrackerGouttefarde tracker3(robot, &setpoint3, &state_estimator);
ControllerTrackerSetpoint<SetpointPurePursuit, TrackerGouttefarde> controller1(
    setpoint1, tracker1);
ControllerTrackerSetpoint<SetpointWaypoints, TrackerGouttefarde> controller2(
    setpoint2, tracker2);
ControllerTrackerSetpoint<SetpointSpline, TrackerGouttefarde> controller3(
    setpoint3, tracker3);

ControllerSwitchable<
    ControllerTrackerSetpoint<SetpointPurePursuit, TrackerGouttefarde>,
    ControllerTrackerSetpoint<SetpointWaypoints, TrackerGouttefarde>,
    ControllerTrackerSetpoint<SetpointSpline, TrackerGouttefarde>>
    controller(controller1, controller2, controller3);
#endif

Odrive odrive(robot, controller);
Estop<ESTOP> estop(odrive, &controller, spray);
Debug debug(SerialD, robot, &controller, &state_estimator, odrive, spray);
Debug computer(SerialComputer, robot, &controller, &state_estimator, odrive,
               spray, 100);
Arm<usb_serial3_class> arm(SerialComputerArm, ArmSerial);
Slave slave(Serial);

// -------------------------------------------------------------
void setup(void) {
  robot.setup();
  state_estimator.setup();
  controller.setup();
  odrive.setup();
  spray.setup();
  arm.setup();
  estop.setup();
  debug.setup();
  computer.setup();
  slave.setup();
}

// -------------------------------------------------------------
void loop(void) {
  estop.update();
  state_estimator.update();
  robot.update();
  controller.update();
  odrive.update();
  spray.update();
  arm.update();
  slave.update();
  debug.update();
  computer.update();
}
