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

#include "can_simple.h"

#ifndef __MK66FX1M0__
  #error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif

#include "DataTypes.h"
#include "SerialComm.h"
#include "OdriveComm.h"
#include "Controller.h"

SerialComm serial(Serial), serial1(Serial1);
Axis axis[4];
OdriveComm odrv(Can0, serial1, axis);
Controller controller(axis);

// -------------------------------------------------------------
void setup(void)
{
  delay(1000);
  Serial.begin(115200);
  Serial1.begin(115200);

  Can0.begin(500000);
  Can0.setNumTXBoxes(8);

  serial.setOdrv(&odrv);
  serial1.setOdrv(&odrv);
  serial.setController(&controller);
  serial1.setController(&controller);
  serial.setAxis(axis);
  serial1.setAxis(axis);

  pinMode(13, OUTPUT);

  for (int i = 0; i < 4; i++) {
    axis[i].setId(i);
    axis[i].setOdrv(&odrv);
  }
}

// -------------------------------------------------------------
void loop(void)
{
  serial.read();
  serial1.read();
  odrv.read();
  controller.update();
  odrv.sendCanBuffer();
}

