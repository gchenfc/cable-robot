#pragma once

#include <BasicStepperDriver.h>

#define BT_RX A4
#define BT_TX A5
#define BT_STATE 2

#define DIR1 11
#define STEP1 10
#define MICROSTEPS1 16
#define STEPS_PER_REV1 -200
float rpm1 = 150;

#define DIR2 3
#define STEP2 5
#define P_EN2 4
#define P_CLK2 6
#define P_PDN A3
#define P_PDN_ALT 7
#define MICROSTEPS2 16
#define STEPS_PER_REV2 200
float rpm2 = 2;

BasicStepperDriver stepper1(STEPS_PER_REV1, DIR1, STEP1);
BasicStepperDriver stepper2(STEPS_PER_REV2, DIR2, STEP2);

int pos1 = 0, pos2 = 0;

#define PEN_OUT_POS 2000
#define PEN_IN_POS 0

class EndEffector {
 private:
  bool ignore_pen_ = true;

 public:
  EndEffector() {}

  void pen_out() {
    if (ignore_pen_) return;
    if (pos1 < PEN_OUT_POS) {
      Serial.println("Moving pen out");
      stepper1.rotate(PEN_OUT_POS - pos1);
      pos1 = PEN_OUT_POS;
    }
  }
  void pen_in() {
    if (ignore_pen_) return;
    if (pos1 > PEN_IN_POS) {
      Serial.println("Moving pen in");
      stepper1.rotate(PEN_IN_POS - pos1);
      pos1 = PEN_IN_POS;
    }
  }

  void set_ignore_pen(bool ignore_pen) { ignore_pen_ = ignore_pen; }

  void set_color(int color) {
    int angle = color * 60;
    if (pos2 != angle) {
      Serial.print("Changing to color ");
      Serial.println(color, DEC);
      int delta = (angle + (180 + 360) - pos2) % 360 - 180;
      stepper2.rotate(delta);
      pos2 = (angle + 720) % 360;
    }
  }

  bool move(int motor, float deg) {
    if (motor == 1) {
      Serial.println("Moving stepper 1 by " + String(deg) + " degrees");
      stepper1.rotate(deg);
      pos1 += deg;
    } else if (motor == 2) {
      Serial.println("Moving stepper 2 by " + String(deg) + " degrees");
      stepper2.rotate(deg);
      pos2 += deg;
    } else {
      return false;
    }
    return true;
  }

  bool move_to(int motor, float deg) {
    if (motor == 1) {
      Serial.println("Moving stepper 1 to " + String(deg) + " degrees");
      stepper1.rotate(deg - pos1);
      pos1 = deg;
    } else if (motor == 2) {
      Serial.println("Moving stepper 2 to " + String(deg) + " degrees");
      stepper2.rotate(deg - pos2);
      pos2 = deg;
    } else {
      return false;
    };
    return true;
  }

  bool set_rpm(int motor, float rpm) {
    if (motor == 1) {
      Serial.println("Setting stepper 1 to " + String(rpm) + " rpm");
      stepper1.setRPM(rpm);
    } else if (motor == 2) {
      Serial.println("Setting stepper 2 to " + String(rpm) + " rpm");
      stepper2.setRPM(rpm);
    } else {
      return false;
    }
    return true;
  }
};
