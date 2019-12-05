/**
 * Teensy code for relaying ODrive stuff to ROS over rosserial.
 * Contains most of base raw functionality from CAN protocol.  Doesn't include fancy functions.
 * Usage:
 *  Upload this code to the Teensy.
 *  Run `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0` (change ttyACM0 with whichever port the Teensy is on)
 * 
 * Authors: Frank Delleart and Gerry Chen and James Luo
 */

#include "ChRt.h"
#include <ros.h>
#include <FlexCAN.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
// #include <std_msgsIMultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "odrvCan.h"
ros::NodeHandle nh;
#include "rosAxis.h"
//------------

#define NUM_DRIVES 4
RosAxis axis[NUM_DRIVES] = {RosAxis(0), RosAxis(1), RosAxis(2), RosAxis(3)};
float pos[NUM_DRIVES] = {0,0,0,0};
float vel[NUM_DRIVES] = {0,0,0,0};

int status =0;
void alive_led_cb( const std_msgs::Empty& toggle_msg){
  status ++;   // blink the led
  digitalWrite(LED_BUILTIN, status % 2);
}
ros::Subscriber<std_msgs::Empty> alive_led("toggle_led", &alive_led_cb);

static CAN_message_t msg, inMsg;
//============

//------------------------------------------------------------------------------
// Thread CanRequest - polls odrives for position, current, and voltage
//
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThreadCanRequest, 64);

static THD_FUNCTION(ThreadCanRequest, arg) {
  (void)arg;
  while (true) {
    for (int i=0; i<NUM_DRIVES; i++){
      requestInfo(i, MSG_GET_ENCODER_ESTIMATES);
      requestInfo(i, MSG_GET_IQ);
      requestInfo(i, MSG_GET_VBUS_VOLTAGE);
      chThdSleepMilliseconds(5);
    }
  }
}
//------------------------------------------------------------------------------
// Thread CanRead - read CAN messages from odrive
//
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThreadCanRead, 64);

static THD_FUNCTION(ThreadCanRead, arg) {
  (void)arg;

  while (true) {
    if (Can0.available()) {
      Can0.read(inMsg);
      uint16_t nodeID = inMsg.id >> 5;
      if (nodeID >= NUM_DRIVES) {
        continue;
      }
      switch(inMsg.id & 0x1F) {
        case CanMsg_t::MSG_ODRIVE_HEARTBEAT:
          axis[nodeID].publishError(*(int32_t*)(inMsg.buf));
          axis[nodeID].publishCurState(*(int32_t*)(&inMsg.buf[4]));
          break;
        case CanMsg_t::MSG_GET_ENCODER_ESTIMATES:
          pos[nodeID] = *(float*)(inMsg.buf); // float32_t does not exist :(
          vel[nodeID] = *(float*)(&inMsg.buf[4]); // float32_t does not exist :(
          axis[nodeID].publishCurPos(pos[nodeID]);
          axis[nodeID].publishCurVel(vel[nodeID]);
          break;
        case MSG_GET_IQ:
          axis[nodeID].publishCurSetCur(*(float*)(&inMsg.buf));
          axis[nodeID].publishCurCur(*(float*)(&inMsg.buf[4]));
          break;
        case MSG_GET_VBUS_VOLTAGE:
          axis[nodeID].publishCurVolt(*(float*)(&inMsg.buf));
          break;
      }
    }
    chThdSleepMicroseconds(1);
  }
}
//------------------------------------------------------------------------------
// Thread RosService, ros spin thread
static THD_WORKING_AREA(waThreadRosService, 64);

static THD_FUNCTION(ThreadRosService, arg) {
  (void)arg;

  while (true) {
    nh.spinOnce();
    chThdSleepMilliseconds(1);
  }
}
//------------------------------------------------------------------------------
// continue setup() after chBegin().
void chSetup() {
  // Start threads.
  chThdCreateStatic(waThreadCanRead, sizeof(waThreadCanRead),
    NORMALPRIO + 1, ThreadCanRead, NULL);
  chThdCreateStatic(waThreadCanRequest, sizeof(waThreadCanRequest),
    NORMALPRIO + 1, ThreadCanRequest, NULL);
  chThdCreateStatic(waThreadRosService, sizeof(waThreadRosService),
   NORMALPRIO + 3, ThreadRosService, NULL);
}
//------------------------------------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  setupOdrvCan();
  nh.initNode();
  nh.subscribe(alive_led);
  for (int i=0; i<NUM_DRIVES; i++){
    axis[i].init();
  }
  
  // Initialize OS and then call chSetup.
  chBegin(chSetup);
  // chBegin() resets stacks and should never return.
  while (true) {}
}
//------------------------------------------------------------------------------
// loop() is the main thread.  Not used in this example.
void loop() {
}
