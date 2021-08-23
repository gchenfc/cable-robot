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

#include "constants.h"
#include "robot.h"
#include "controllers/controller_dummy.h"
#include "communication/odrive_can.h"
#include "spray.h"
#include "estop.h"
#include "communication/debug.h"
#include "communication/slave.h"

Robot robot{};
ControllerDummy controller{};
Odrive odrive(robot, controller);
Spray spray(btSerial);
Estop estop(ESTOP);
Debug debug(SerialD);
Slave slave(Serial);

// -------------------------------------------------------------
void setup(void) {
  robot.setup();
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
  robot.update();
  controller.update();
  odrive.update();
  spray.update();
  slave.update();
  debug.update();
}
