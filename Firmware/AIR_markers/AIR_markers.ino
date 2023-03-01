// #include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include "end_effector.h"
#include "command_parser.h"

#define LED 13

// SoftwareSerial BT(BT_RX, BT_TX);
AltSoftSerial BT;


EndEffector end_effector;
SerialParser parser1(Serial, Serial, end_effector);
SerialParser parser2(BT, Serial, end_effector);

void setup() {
  Serial.begin(115200);
  BT.begin(9600);

  pinMode(STEP1, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(P_EN2, OUTPUT);
  stepper1.begin(rpm1, MICROSTEPS1);
  stepper2.begin(rpm2, MICROSTEPS2);
  digitalWrite(P_EN2, LOW);

  pinMode(BT_STATE, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // while (true) {
  //   Serial.println("1");
  //   stepper1.rotate(360);
  //   Serial.println("2");
  //   stepper2.rotate(15);
  //   Serial.println("3");
  //   stepper1.rotate(-360);
  //   Serial.println("4");
  //   stepper2.rotate(-15);
  // }
}

void loop() {
  digitalWrite(LED, digitalRead(BT_STATE));
  parser1.update();
  parser2.update();
  return;
}
