#include "Controller.h"
#include "DataTypes.h"
#include "SerialComm.h"
#include "OdriveComm.h"
#include "math.h"
#include <Metro.h>

void Controller::IK(float lengths[4], float x, float y) const {
  float mountPoints[4][2] = {{width, 0}, {width, height}, {0, height}, {0, 0}};
  for (int i = 0; i < 4; i++) {
    float dx = x - mountPoints[i][0];
    float dy = y - mountPoints[i][1];
    lengths[i] = sqrt(x*x + y*y);
  }
}
void Controller::FK(float &x, float &y, const float lengths[4]) const {
  float a = width;
  float b = lengths[3];
  float c = lengths[0];
  float cosC = (a*a + b*b - c*c) / (2*a*b);
  if ((cosC <= 1) && (cosC >= -1)) {
    x = b * cosC;
    y = b * (1-cosC*cosC);
  } else {
    // kinematically infeasible
    x = 0;
    y = 0;
  }
}
void Controller::forceSolver(float tensions[4], float Fx, float Fy, float x, float y) const {
  float mountPoints[4][2] = {{width, 0}, {width, height}, {0, height}, {0, 0}};
  // first identify which cables to use.
  // The desired force will lie between 2 cables (unless F is 0 in which case it doesn't matter)
  float dx = mountPoints[3][0] - x;
  float dy = mountPoints[3][1] - y;
  float cross1 = dx * Fy - dy * Fx;
  int cablei;
  for (cablei = 0; cablei < 4; cablei++) {
    dx = mountPoints[cablei][0] - x;
    dy = mountPoints[cablei][1] - y;
    float cross2 = dx * Fy - dy * Fx;
    if ((cross1 > 0) && (cross2 <= 0))  // cross product switches from + to -
      break;
    cross1 = cross2;
  }
  // indices to reduce if-trees
  uint8_t cable2 = cablei;
  uint8_t cable1 = (cablei+3) % 4;
  uint8_t notcable1 = (cablei+1) % 4;
  uint8_t notcable2 = (cablei+2) % 4;
  // norms will be useful later
  float norms[4];
  for (int i = 0; i < 4; i++) {
    dx = mountPoints[i][0] - x;
    dy = mountPoints[i][1] - y;
    norms[i] = sqrt(dx*dx + dy*dy);
  }
  // minimum cable torque is 0.2Nm
  tensions[notcable1] = 0.2;
  Fx = Fx + 0.2*(mountPoints[notcable1][0] - x) / norms[notcable1];
  Fx = Fx + 0.2*(mountPoints[notcable2][0] - x) / norms[notcable2];
  tensions[notcable2] = 0.2;
  Fy = Fy + 0.2*(mountPoints[notcable1][1] - y) / norms[notcable1];
  Fy = Fy + 0.2*(mountPoints[notcable2][1] - y) / norms[notcable2];
  // solve for:
  // (m1-x)*f1/n1 + (m2-x)*f2/n2 = F
  //    where m1/m2 are mounting points, n1/n2 are normalizing norms, and f1/f2 are cable tensions
  // [(m1x-x)/n1  (m2x-x)/n2].[f1] = [Fx] 
  // [(m1y-y)/n1  (m2y-y)/n2] [f2]   [Fy]
  float DX[2][2] = {{(mountPoints[cable1][0] - x) / norms[cable1], (mountPoints[cable2][0] - x) / norms[cable2]},
                    {(mountPoints[cable1][1] - y) / norms[cable1], (mountPoints[cable2][1] - y) / norms[cable2]}};
  // 2x2 inverse
  float det = DX[0][0]*DX[1][1] - DX[1][0]*DX[0][1];
  float DXinv[2][2] = {{DX[1][1]/det, -DX[0][1]/det},
                       {-DX[1][0]/det, DX[0][0]/det}};
  // solve
  tensions[cable1] = DXinv[0][0]*Fx + DXinv[0][1];
  tensions[cable2] = DXinv[1][0]*Fx + DXinv[1][1]*Fy;
}

/* Usage: easiest to just copy/paste into godbolt
void testForceSolver() {
  float x = 1, y = 1;
  float width = 3, height = 2;
  float Fx = 1, Fy = 1;
  uint8_t expected_cable1 = 1;
  uint8_t expected_cable2 = 2;
  float expected[4] = {0.2, 1.346560, 0.236083, 0.2};
  float actual[4];
  forceSolver(actual, Fx, Fy, x, y, width, height);
  cout << "Cable 1: " << (int)expected_cable1 << '\t';
  cout << "Cable 2: " << (int)expected_cable2 << endl;
  cout << actual[0] << ' '
       << actual[1] << ' '
       << actual[2] << ' '
       << actual[3] << ' ' << endl;
  cout << expected[0] << ' ' <<
          expected[1] << ' ' <<
          expected[2] << ' ' <<
          expected[3] << ' ' << endl;
}
*/

void Controller::update() {
  if (controllerTimer.check()) {
    switch (state_) {
      case CTRL_STATE_IDLE:
        return;
      case CTRL_STATE_GOING:
        cartesianPositionControl();
        break;
    }
  }
}

void Controller::processMsg(const Msg_t &msg, SerialComm &serial) {
  uint32_t axis;
  float x, y;
  switch (msg.cmd) {
    case CTRL_CMDS_SET_HEIGHTWIDTH:
      msg.getFloat(height, width);
      break;
    case CTRL_CMDS_GET_HEIGHTWIDTH:
      outMsg_.cmd = CTRL_CMDS_GET_HEIGHTWIDTH;
      outMsg_.setFloat(height, width);
      serial.write(outMsg_);
      break;
    case CTRL_CMDS_GOTO_X:
      outMsg_.getFloat(setpointx, setpointy);
      requesteState(CTRL_STATE_GOING);
      break;
    case CTRL_CMDS_GET_X:
      outMsg_.cmd = CTRL_CMDS_GET_X;
      FK(x, y);
      outMsg_.setFloat(x, y);
      serial.write(outMsg_);
      break;
  }
}

void Controller::requesteState(ControllerState_t state) {
  if (state_ == CTRL_STATE_IDLE) {
    state_ = state;
  }
}

void Controller::cartesianPositionControl() {
  float x, y;
  FK(x, y);
  float dx = setpointx - x;
  float dy = setpointy - y;
  float gain = 0.2;
  float Fx = dx * gain;
  float Fy = dy * gain;
  float tensions[4];
  forceSolver(tensions, Fx, Fy, x, y);
  for (int i = 0; i < 4; i++) {
    axis_[i].setTorque(tensions[i]);
  }
}