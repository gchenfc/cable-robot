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

// function declarations
uint8_t requestInfo(uint16_t node, uint8_t cmdId, bool rtr=false);
uint8_t sendBytes(uint16_t node, uint8_t cmdId, uint8_t *data, bool rtr=false);
uint8_t sendInt32(uint16_t node, uint8_t cmdId, int32_t d1, int32_t d2 = 0);
uint8_t sendFloat(uint16_t node, uint8_t cmdId, float d1, float d2 = 0);
uint8_t sendInputPos(uint16_t node, uint8_t cmdId, float pos, int16_t vel, int16_t torque);
void start_closed_loop(uint8_t node);
void stop_closed_loop(uint8_t node);
#include "serial.h"

// global variables
Metro queryTimer(5);
CAN_message_t inMsg;
uint64_t frame;
uint64_t sframe;
uint8_t countThisFrame;

volatile bool estopStatus = false;
Metro estopTimer(50);

Metro printTimer(50);
uint32_t status[4], state[4];
float pos[4], vel[4], iqset[4], iqmeas[4], tset[4];

bool closed[4];
bool closed2;
float pos_set[4];
float vel_lpf, errI;
uint64_t closed2_toff, c2_tlast;
float t, xdes, vdes;

static float T = 2, A = 0.5, tau = 2*3.1415, r = 0.0254/2;

// -------------------------------------------------------------
void setup(void)
{
  // delay(1000);
  Serial1.begin(115200);
  Serial1.println("Hello Teensy 3.6 dual CAN Test.");
  pinMode(13, OUTPUT);

  Can0.begin(1000000);

  Can0.setNumTXBoxes(8);
  frame = 0;
  sframe = 0;
  pinMode(ESTOP, INPUT_PULLDOWN);
  attachInterrupt(ESTOP, estop, FALLING);

  memset(closed, 0, 4);
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
  while (true) {
    while (Can0.available()) {
      Can0.read(inMsg);
      if ((inMsg.id & 0x1F) == MSG_GET_ENCODER_ESTIMATES) {
        // Serial1.print(int(sframe));
        // Serial1.print('\t');
        // Serial1.print(int(frame));
        // Serial1.print('\t');
        // Serial1.print(millis());
        // Serial1.print('\t');
        // Serial1.print(inMsg.id >> 5, HEX);
        // Serial1.print('\t');
        // Serial1.print(inMsg.id & 0x1F, HEX);
        // Serial1.print('\t');
        float f1, f2;
        read_float(inMsg.buf, &f1, &f2);
        // Serial1.print(f1);
        // Serial1.print('\t');
        // Serial1.print(f2);
        // // for (auto d : inMsg.buf) {
        // //   Serial1.print(d);
        // //   Serial1.print(", ");
        // // }
        // Serial1.println();
        // sframe++;
        // countThisFrame++;
        uint8_t node = inMsg.id >> 5;
        pos[node] = f1;
        vel[node] = f2;

        if (closed[node]) {
          float e = pos_set[node] - pos[node];
          float de = 0 - vel[node];
          float torque = max(0.1 * e, .1) + 0.03*de;
          sendFloat(node, 0x0E, torque);
          tset[node] = torque;
        }

      } else if ((inMsg.id & 0x1F) == MSG_ODRIVE_HEARTBEAT) {
        // Serial1.print(millis());
        // Serial1.print('\t');
        // Serial1.print(inMsg.id, HEX);
        // Serial1.print('\t');
        int i1, i2;
        read_int32(inMsg.buf, &i1, &i2);
        // Serial1.print(i1);
        // Serial1.print('\t');
        // Serial1.print(i2);
        // Serial1.println();
        status[inMsg.id >> 5] = i1;
        state[inMsg.id >> 5] = i2;
      } else {
        // Serial1.println(inMsg.id);
      }
    }

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

      if (queryTimer.check()){
        frame += 1;
        requestInfo(0, MSG_GET_ENCODER_ESTIMATES, true);
        requestInfo(1, MSG_GET_ENCODER_ESTIMATES, true);
        requestInfo(2, MSG_GET_ENCODER_ESTIMATES, true);
        requestInfo(3, MSG_GET_ENCODER_ESTIMATES, true);
        // Serial1.println(countThisFrame);
        countThisFrame = 0;

        if (closed2) {
          t = ((millis() - closed2_toff) % static_cast<uint64_t>(1000 * T)) /
              1000.0;
          xdes = A * sinf(tau * t / T); // m
          vdes = A * (tau / T) * cosf(tau * t / T); // m/s
          // FK
          float x = (pos[0] - pos_set[0] - pos[2] + pos_set[2]) / 2 * r * tau;
          float v = (vel[0] - vel[2]) / 2 * r * tau;
          // PID
          vel_lpf = 0.05 * (v - vel_lpf) + vel_lpf;
          uint64_t tnow = millis();
          float dt = (tnow - c2_tlast) / 1000.0;
          c2_tlast = tnow;
          float err = xdes - x;
          errI += min(.5, max(-.5, err * dt));
          float derr = vdes - vel_lpf;
          float force = 2000 * err + 1000 * errI + 500 * derr;
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
      }
    }

    read_serial();

    if (printTimer.check()) {
      for (auto s : status) {
        Serial1.print(s, HEX);
        Serial1.print(',');
      }
      for (auto s : state) {
        Serial1.print(s, HEX);
        Serial1.print(',');
      }
      for (auto p : pos) {
        Serial1.print(p);
        Serial1.print('\t');
      }
      for (auto v : vel) {
        Serial1.print(v);
        Serial1.print('\t');
      }
      for (auto t : tset) {
        Serial1.print(t);
        Serial1.print('\t');
      }
      float pos2 = (pos[0] - pos_set[0] - pos[2] + pos_set[2]) / 2 * r * tau;
      Serial1.print(pos2);
      Serial1.print(',');
      Serial1.print(xdes);
      Serial1.print(',');
      Serial1.print(errI);
      Serial1.println();
    }
  }
}

void start_closed_loop(uint8_t node) {
  pos_set[node] = pos[node];
  closed[node] = true;
}
void stop_closed_loop(uint8_t node) {
  closed[node] = false;
  sendFloat(node, 0x0E, 0);
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

uint8_t requestInfo(uint16_t node, uint8_t cmdId, bool rtr) {
  uint8_t data[8];
  memset(data, 0, 8);
  return sendBytes(node, cmdId, data, rtr);
}

void read_float(uint8_t *data, float *float1, float *float2) {
  memcpy(float1, data, 4);
  memcpy(float2, data+4, 4);
}

void read_int32(uint8_t *data, int32_t *i1, int32_t *i2) {
  memcpy(i1, data, 4);
  memcpy(i2, data+4, 4);
}

uint8_t sendInt32(uint16_t node, uint8_t cmdId, int32_t d1, int32_t d2) {
  uint8_t data[8];
  memcpy(&data, (uint8_t*) (&d1), 4);
  memcpy(((uint8_t *)&data)+4, (uint8_t*) (&d2), 4);
  return sendBytes(node, cmdId, data);
}

uint8_t sendFloat(uint16_t node, uint8_t cmdId, float d1, float d2) {
  uint8_t data[8];
  memcpy(&data, (uint8_t*) (&d1), 4);
  memcpy(((uint8_t *)&data)+4, (uint8_t*) (&d2), 4);
  if (cmdId == 0x0E)
    tset[node] = d1;
  return sendBytes(node, cmdId, data);
}

uint8_t sendInputPos(uint16_t node, uint8_t cmdId, float pos, int16_t vel, int16_t torque) {
  uint8_t data[8];
  memcpy(&data, (uint8_t*) (&pos), 4);
  memcpy(((uint8_t *)&data)+4, (uint8_t*) (&vel), 2);
  memcpy(((uint8_t *)&data)+6, (uint8_t*) (&torque), 2);
  return sendBytes(node, cmdId, data);
}

uint8_t sendBytes(uint16_t node, uint8_t cmdId, uint8_t *data, bool rtr) {
  CAN_message_t outMsg;
  outMsg.id = cmdId | (node << 5);
  outMsg.rtr = rtr;
  outMsg.len = 8;
  outMsg.ext = 0;
  memcpy(outMsg.buf, data, 8);
  uint8_t success = Can0.write(outMsg);
  return success;
}
