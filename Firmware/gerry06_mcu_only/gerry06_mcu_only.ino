// -------------------------------------------------------------
// CANtest for Teensy 3.6 dual CAN bus
// by Collin Kidder, Based on CANTest by Pawelsky (based on CANtest by teachop)
//
// Both buses are left at default 250k speed and the second bus sends frames to the first
// to do this properly you should have the two buses linked together. This sketch
// also assumes that you need to set enable pins active. Comment out if not using
// enable pins or set them to your correct pins.
//
// This sketch tests both buses as well as interrupt driven Rx and Tx. There are only
// two Tx buffers by default so sending 5 at a time forces the interrupt driven system
// to buffer the final three and send them via interrupts. All the while all Rx frames
// are internally saved to a software buffer by the interrupt handler.
//

#include <FlexCAN.h>
#include <Metro.h>

#include "can_simple.h"

#ifndef __MK66FX1M0__
  #error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif

#define ESTOP 24
#define btSerial Serial2

// query variables
Metro queryTimer(10);
CAN_message_t inMsg;

// estop
Metro estopTimer(50);
volatile bool estopStatus = false;

// state management
Metro printTimer(50);
uint32_t status[4], state[4];
float pos[4], vel[4], iqset[4], iqmeas[4], tset[4];
float zeros[4] = {61.59, 27.46, 24.04, 21.18};
// float width = 2.84, height = 2.44;
// float width = 37 * (3.1415*0.0254), height = 27 * (3.1415*0.0254);
float width = 3.02, height = 2.3;
// width = 3.048

// control stuff
bool closed[4];
bool closed2, closed4, closed4t;
float pos_set[4];
float vel_lpf, errI;
uint64_t closed2_toff, c2_tlast;
float t, xdes, vdes;

float t4, x4start, y4start, x4, y4, x4des, y4des, vx4des, vy4des, ax4des, ay4des, FFx, FFy;
float qdot4des[4], qddot4des[4];
float vel4_lpf[2], err4I[2];
uint64_t c4_toff, c4_tlast;
float W[4][2];

uint32_t ct4_ind;
float ct4_xset, ct4_yset;
bool ct4_proceed, ct4_run, ct4_keepind = false;
bool setpaint;
uint8_t setcolor;

bool manualpaint = false;

// "constants"
static float T = 5, A = 0.50, tau = 2*3.1415, r = 0.0254/2;

// function declarations
#include "can_utils.h"
#include "kinematic_utils.h"
void start_closed_loop_1(uint8_t node);
void stop_closed_loop_1(uint8_t node);
void update_control(uint8_t node);
void start_closed_loop_2();
void stop_closed_loop_2();
void update_control_2();
void printInfo();
#include "serial.h"
// #include "iros_logo.h" // trajectory
#include "step.h" // trajectory

// -------------------------------------------------------------
void setup(void)
{
  Serial1.begin(115200);
  pinMode(13, OUTPUT);

  // CAN
  Can0.begin(1000000);
  Can0.setNumTXBoxes(8);
  // estop
  pinMode(ESTOP, INPUT_PULLDOWN);
  attachInterrupt(ESTOP, estop, FALLING);
  // bluetooth
  btSerial.begin(9600);
  // initialization
  memset(closed, 0, 4);
  closed2 = false;
  estopStatus = false;

  #ifdef TRAJ_SETUP
  setup_traj();
  #endif
}

void estop() {
  requestInfo(0, 0x002, false);
  requestInfo(1, 0x002, false);
  requestInfo(2, 0x002, false);
  requestInfo(3, 0x002, false);
  estopStatus = true;
  Serial1.println("ESTOPPED");
}

// -------------------------------------------------------------
void loop(void)
{
  while (Can0.available()) {
    Can0.read(inMsg);
    uint8_t node = inMsg.id >> 5;
    if ((inMsg.id & 0x1F) == MSG_GET_ENCODER_ESTIMATES) {
      read_float(inMsg.buf, &pos[node], &vel[node]);
      if (closed[node]) {
        update_closed_1(node);
      }

    } else if ((inMsg.id & 0x1F) == MSG_ODRIVE_HEARTBEAT) {
      read_int32(inMsg.buf, &status[node], &state[node]);
    } else {
      // Serial1.println(inMsg.id);
    }
  }

  // handle estop
  if (estopStatus) {
    if (digitalRead(ESTOP)) // good now
      estopStatus = false;
    else if (estopTimer.check())
      estop();
  } else {
    if (!digitalRead(ESTOP))
      estop();
    else
      estopStatus = false;
  }

  // query timer
  if (!estopStatus && queryTimer.check()){
    requestInfo(0, MSG_GET_ENCODER_ESTIMATES, true);
    requestInfo(1, MSG_GET_ENCODER_ESTIMATES, true);
    requestInfo(2, MSG_GET_ENCODER_ESTIMATES, true);
    requestInfo(3, MSG_GET_ENCODER_ESTIMATES, true);
    if (closed2) {
      update_control_2();
    }
    FK(x4, y4);
    if (closed4) {
      update_control_4();
    }
    if (closed4t) {
      update_control_4_traj();
    }
  }

  // serial
  read_serial();

  // print
  if (printTimer.check()) {
    printInfo();
  }
}

void start_closed_loop_1(uint8_t node) {
  pos_set[node] = pos[node];
  closed[node] = true;
}
void stop_closed_loop_1(uint8_t node) {
  closed[node] = false;
  sendFloat(node, 0x0E, 0);
}
void update_closed_1(uint8_t node) {
  float e = pos_set[node] - pos[node];
  float de = 0 - vel[node];
  float torque = max(0.1 * e, .1) + 0.03*de;
  sendFloat(node, 0x0E, torque);
  tset[node] = torque;
}

void start_closed_loop_2() {
  pos_set[0] = pos[0];
  pos_set[2] = pos[2];
  closed2 = true;
  vel_lpf = 0;
  errI = 0;
  c2_tlast = millis();
  closed2_toff = millis();
}
void stop_closed_loop_2() {
  closed2 = false;
  sendFloat(0, 0x0E, 0);
  sendFloat(2, 0x0E, 0);
}

void update_control_2() {
  t = ((millis() - closed2_toff) % static_cast<uint64_t>(1000 * T)) /
      1000.0;
  xdes = A * sinf(tau * t / T); // m
  vdes = A * (tau / T) * cosf(tau * t / T); // m/s
  // FK
  float x = (pos[0] - pos_set[0] - pos[2] + pos_set[2]) / 2 * r * tau;
  float v = (vel[0] - vel[2]) / 2 * r * tau;
  // PID
  vel_lpf = 0.1 * (v - vel_lpf) + vel_lpf;
  uint64_t tnow = millis();
  float dt = (tnow - c2_tlast) / 1000.0;
  c2_tlast = tnow;
  float err = xdes - x;
  errI += err * dt;
  errI = min(.5, max(-.5, errI));
  float derr = vdes - vel_lpf;
  float force = 1000 * err + 500 * errI + 250 * derr;
  // IK
  float torque[2] = {0.2, 0.2};
  if (force > 0) {  
    torque[0] += force * r;
  } else {
    torque[1] += abs(force) * r;
  }
  sendFloat(0, 0x0E, torque[0]);
  sendFloat(2, 0x0E, torque[1]);
}

void start_closed_loop_4() {
  FK(x4start, y4start);
  x4start -= A;
  jacobian(W);
  FKv(W, vel4_lpf[0], vel4_lpf[1]);
  err4I[0] = 0; err4I[1] = 0;
  c4_tlast = millis();
  c4_toff = millis();
  closed4 = true;
}
void stop_closed_loop_4() {
  closed4 = false;
  sendFloat(0, 0x0E, 0);
  sendFloat(1, 0x0E, 0);
  sendFloat(2, 0x0E, 0);
  sendFloat(3, 0x0E, 0);
}
void update_control_4() {
  t = ((millis() - c4_toff) % static_cast<uint64_t>(1000 * T)) /
      1000.0;
  x4des = A * cosf(tau * t / T) + x4start; // m
  y4des = A * sinf(tau * t / T) + y4start;
  vx4des = -A * (tau / T) * sinf(tau * t / T); // m/s
  vy4des = A * (tau / T) * cosf(tau * t / T);
  // Serial1.print(x4des);
  // Serial1.print(", ");
  // Serial1.print(y4des);
  // Serial1.print('\t');
  // FK
  // Serial1.print(getLen(0));
  // Serial1.print(", ");
  // Serial1.print(getLen(1));
  // Serial1.print(", ");
  // Serial1.print(getLen(2));
  // Serial1.print(", ");
  // Serial1.print(getLen(3));
  // Serial1.print('\t');
  // FK(x4, y4);
  // Serial1.print(x4);
  // Serial1.print(", ");
  // Serial1.print(y4);
  // Serial1.print('\t');

  jacobian(W);
  float vx, vy;
  FKv(W, vx, vy);
  // PID
  LPF(vel4_lpf[0], vx);
  LPF(vel4_lpf[1], vy);
  uint64_t tnow = millis(); float dt = (tnow - c4_tlast) / 1000.0; c4_tlast = tnow;
  float errx = x4des - x4, erry = y4des - y4;
  err4I[0] += errx * dt; err4I[1] += erry * dt;
  err4I[0] = min(0.5, max(-0.5, err4I[0]));     err4I[1] = min(0.5, max(-0.5, err4I[1]));
  float derrx = vx4des - vel4_lpf[0], derry = vy4des - vel4_lpf[1];
  // float fx = 1000 * errx + 500 * err4I[0] + 250 * derrx;
  // float fy = 1000 * erry + 500 * err4I[1] + 250 * derry;
  float fx = 400.0 * errx + 200.0 * err4I[0] + 100.0 * derrx;
  float fy = 400.0 * erry + 200.0 * err4I[1] + 100.0 * derry;
  // Serial1.print(fx);
  // Serial1.print(", ");
  // Serial1.print(fy);
  // Serial1.print('\t');
  // IK
  float tensions[4];
  forceSolver(tensions, fx, fy);
  // Serial1.print(tensions[0]);
  // Serial1.print(", ");
  // Serial1.print(tensions[1]);
  // Serial1.print(", ");
  // Serial1.print(tensions[2]);
  // Serial1.print(", ");
  // Serial1.print(tensions[3]);
  // Serial1.print('\n');
  for (uint8_t i = 0; i < 4; ++i)
    sendFloat(i, 0x0E, tensions[i] * r);
}

void updateSetpoint() {
  ct4_xset = (traj[ct4_ind][0] - width/2) * TRAJ_SCALE + width/2;
  ct4_yset = (traj[ct4_ind][1] - height/2) * TRAJ_SCALE + height/2;
  setpaint = painton[ct4_ind];
  setcolor = colorinds[ct4_ind];
}
void start_closed_loop_4_traj() {
  jacobian(W);
  FKv(W, vel4_lpf[0], vel4_lpf[1]);
  err4I[0] = 0; err4I[1] = 0;
  c4_tlast = millis();
  if (!ct4_keepind)
    ct4_ind = 0;
  else
    ct4_keepind = false;
  updateSetpoint();
  x4des = ct4_xset; y4des = ct4_yset; // needed for derivative calc
  vx4des = 0; vy4des = 0; // needed for derivative calc
  memset(qdot4des, 0, sizeof(qdot4des)); // needed for derivative calc
  ax4des = 0; ay4des = 0; // LPF
  FFx = 0; FFy = 0;
  ct4_proceed = false;
  ct4_run = true;
  closed4t = true;
}
void step_closed_loop_4_traj() {
  if (!closed4t)
    start_closed_loop_4_traj();
  else
    ct4_proceed = true;
  ct4_run = false;
}
void seti_closed_loop_4_traj(uint16_t ind) {
  if (closed4t)
    return; // too dangerous 
  if (ind >= (sizeof(traj) / sizeof(traj[0])))
    return;
  ct4_ind = ind;
  ct4_keepind = true;
  Serial1.print("Set trajectory index to ");
  Serial1.println(ct4_ind);
}
void stop_closed_loop_4_traj() {
  closed4t = false;
  sendFloat(0, 0x0E, 0);
  sendFloat(1, 0x0E, 0);
  sendFloat(2, 0x0E, 0);
  sendFloat(3, 0x0E, 0);
  ct4_ind = 0;
}
void update_control_4_traj() {
  // update timing
  uint64_t tnow = millis(); float dt = (tnow - c4_tlast) / 1000.0; c4_tlast = tnow;
  // update setpoint
  updateSetpoint();
  ct4_proceed |= ct4_run;
  if (ct4_proceed) {
    ++ct4_ind;
    if (ct4_ind >= (sizeof(traj) / sizeof(traj[0]))) {
      #ifdef TRAJ_LOOP
      ct4_ind = 0;
      #else
      stop_closed_loop_4_traj();
      return;
      #endif
    }
    ct4_proceed = false;
  }
  float prev_xdes = x4des, prev_ydes = y4des;
  x4des = ct4_xset;
  y4des = ct4_yset;
  float prev_vx4des = vx4des, prev_vy4des = vy4des;
  vx4des = (x4des - prev_xdes) / dt; // m/s
  vy4des = (y4des - prev_ydes) / dt;
  ax4des = (vx4des - prev_vx4des) / dt;
  ay4des = (vy4des - prev_vy4des) / dt;
  for (uint8_t i = 0; i < 4; ++i) {
    float prev_qdot4des = qdot4des[i];
    qdot4des[i] = W[i][0] * vx4des + W[i][1] * vy4des;
    qddot4des[i] = (qdot4des[i] - prev_qdot4des) / dt;
    clamp(qddot4des[i], -100, 100);
  }
  // vx4des = 0;
  // vy4des = 0;

  // calculate current velocity
  jacobian(W);
  float vx, vy;
  FKv(W, vx, vy);

  // PID
  LPF(vel4_lpf[0], vx);
  LPF(vel4_lpf[1], vy);
  float errx = x4des - x4, erry = y4des - y4;
  clamp(errx, -0.25, 0.25); clamp(erry, -0.25, 0.25);
  err4I[0] += errx * dt; err4I[1] += erry * dt;
  clamp(err4I[0], -0.5, 0.5); clamp(err4I[1], -0.5, 0.5);
  float derrx = vx4des - vel4_lpf[0], derry = vy4des - vel4_lpf[1];
  clamp(derrx, -0.5, 0.5); clamp(derry, -0.5, 0.5);
  // feedforward
  FFx += 0.1 * ax4des, FFy += 0.1 * ay4des;
  float FFxAction = FFx, FFyAction = FFy;
  clamp(FFxAction, -100, 100); clamp(FFyAction, -100, 100);
  FFx -= FFxAction; FFy -= FFyAction;
  FFx = 0; FFy = 0;
  // float fx = 1000 * errx + 500 * err4I[0] + 250 * derrx;
  // float fy = 1000 * erry + 500 * err4I[1] + 250 * derry;
  float fx = 400.0 * errx + 200.0 * err4I[0] + 100.0 * derrx + FFxAction;
  float fy = 400.0 * erry + 200.0 * err4I[1] + 100.0 * derry + FFyAction;
  // float fx = 200.0 * errx + 100.0 * err4I[0] + 10.0 * derrx;
  // float fy = 200.0 * erry + 100.0 * err4I[1] + 10.0 * derry;
  clamp(fx, -100, 100); clamp(fy, -100, 100);

  // TD
  float tensions[4];
  forceSolver(tensions, fx, fy);
  for (uint8_t i = 0; i < 4; ++i)
    sendFloat(i, 0x0E, tensions[i] * r);

  // Serial1.print("Traj: ");
  // Serial1.print(ct4_ind); Serial1.print('\t');
  // Serial1.print(x4); Serial1.print(", ");
  // Serial1.print(y4); Serial1.print(", ");
  // Serial1.print(x4des); Serial1.print(", ");
  // Serial1.print(y4des); Serial1.print(", ");
  // Serial1.print(setpaint); Serial1.print(", ");
  // Serial1.print(setcolor); Serial1.print(", ");
  // Serial1.print(sqrt(errx*errx + erry*erry)); Serial1.print(", ");
  // Serial1.print(fx);
  // Serial1.print(", ");
  // Serial1.print(fy);
  // Serial1.print('\t');
  // Serial1.print(tensions[0]);
  // Serial1.print(", ");
  // Serial1.print(tensions[1]);
  // Serial1.print(", ");
  // Serial1.print(tensions[2]);
  // Serial1.print(", ");
  // Serial1.print(tensions[3]);
  // Serial1.print('\n');
}

void printInfo() {
  for (auto s : status) {
    Serial1.print(s, HEX);
    Serial1.print(',');
  }
  for (auto s : state) {
    Serial1.print(s, HEX);
    Serial1.print(',');
  }
  // for (auto p : pos) {
  //   Serial1.print(p);
  //   Serial1.print(", ");
  // } Serial1.print('\t');
  for (uint8_t i = 0; i < 4; ++i) {
    Serial1.print(getLen(i));
    Serial1.print(", ");
  } Serial1.print('\t');
  for (auto v : vel) {
    if (v >= 0) Serial1.print(' ');
    if (abs(v) < 10) Serial1.print(' ');
    Serial1.print(v);
    Serial1.print(", ");
  } Serial1.print('\t');
  for (auto t : tset) {
    Serial1.print(t);
    Serial1.print(", ");
  } Serial1.print('\t');
  Serial1.print(ct4_ind);
  Serial1.print(",\t");
  Serial1.print(setpaint); Serial1.print(", ");
  Serial1.print(setcolor); Serial1.print(", ");
  Serial1.print(x4);
  Serial1.print(',');
  Serial1.print(y4);
  Serial1.print(", ");
  Serial1.print(x4des);
  Serial1.print(',');
  Serial1.print(y4des);
  Serial1.print(", ");
  float errx = x4des - x4, erry = y4des - y4;
  Serial1.print(sqrt(errx*errx + erry*erry));
  Serial1.println();

  if ((setpaint && closed4t) || manualpaint)
    btSerial.write('1');
  else 
    btSerial.write('0');
}
