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
// #include "src/controllers/controller_simple.h"
#include "src/controllers/controller_tracking.h"
// #include "src/controllers/controller_ilqr.h"
#include "src/communication/odrive_can.h"
#include "src/spray.h"
#include "src/estop.h"
#include "src/communication/debug.h"
#include "src/communication/slave.h"

Spray spray(btSerial);
Robot robot{};
StateEstimatorFirstOrder state_estimator(robot);
// ControllerSimple controller(&state_estimator);
ControllerTracking controller(&state_estimator);
// ControllerIlqr controller(&state_estimator, spray);
Odrive odrive(robot, controller);
Estop<ESTOP> estop(odrive, &controller, spray);
Debug debug(SerialD, robot, &controller, &state_estimator, odrive, spray,
            [](char *buffer, int size) {
              return human_serial::parseMsgTracking(controller, buffer, size);
            });
Slave slave(Serial);

// -------------------------------------------------------------
void setup(void) {
  robot.setup();
  state_estimator.setup();
  controller.setup();
  odrive.setup();
  spray.setup();
  estop.setup();
  debug.setup();
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
  slave.update();
  debug.update();
}
