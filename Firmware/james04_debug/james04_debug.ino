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
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
// #include <std_msgsIMultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "params.h"
#include "odrvCan.h"
ros::NodeHandle_<ArduinoHardware, 50, 50, 512, 512> nh; // <max subs, max pubs, input size, output size> - see ros_lib/ros/node_handle.h
#include "rosAxis.h"
RosAxis axis[NUM_DRIVES] = {RosAxis(0), RosAxis(1), RosAxis(2), RosAxis(3)};
#include "CableController.h"
//------------

CableController controller;

int alive_led_status = 0;
void alive_led_cb( const std_msgs::Empty& toggle_msg){
  alive_led_status ++;   // blink the led
  digitalWrite(LED_BUILTIN, alive_led_status % 2);
}
ros::Subscriber<std_msgs::Empty> alive_led("toggle_led", &alive_led_cb);

static CAN_message_t msg, inMsg;
//============

//------------------------------------------------------------------------------
// Thread Controller - handles higher level control functionality running on Teensy
//
// 256 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThreadController, 256);
static THD_FUNCTION(ThreadController, arg) {
  (void)arg;
  systime_t updateTime = chVTGetSystemTime(); // Current system time.
  while (true) {
    controller.update();
    updateTime += TIME_MS2I(10);                  // From Milliseconds to Tick conversion (100Hz)
    chThdSleepUntil(updateTime);
  }
}
//------------------------------------------------------------------------------
// Thread ControllerPublisher - periodically reports CableController information
//
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThreadControllerPublisher, 64);
static THD_FUNCTION(ThreadControllerPublisher, arg) {
  (void)arg;
  systime_t updateTime2 = chVTGetSystemTime();
  while (true) {
    controller.publish();
    chThdSleepMilliseconds(50);
  }
}

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
      chThdSleepMilliseconds(40 / NUM_DRIVES); // 25 Hz
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

  uint16_t nodeID;

  while (true) {
    if (Can0.available()) {
      Can0.read(inMsg);
      nodeID = inMsg.id >> 5;
      if (nodeID >= NUM_DRIVES) {
        continue;
      }
      axis[nodeID].readCAN(inMsg);
    }
    resendMsgs(); // service manual TX buffer
    chThdSleepMicroseconds(200);
  }
}
//------------------------------------------------------------------------------
// Thread RosService, ros spin thread
static THD_WORKING_AREA(waThreadRosService, 64);
static THD_FUNCTION(ThreadRosService, arg) {
  (void)arg;

  while (true) {
    nh.spinOnce();
    // chThdSleepMilliseconds(1);
    chThdSleepMicroseconds(200);
  }
}
//------------------------------------------------------------------------------
// Thread RosService, ros spin thread
static THD_WORKING_AREA(waThreadKernelReport, 64);
static THD_FUNCTION(ThreadKernelReport, arg) {
  (void)arg;

  while (true) {
    Serial1.println("lock ");
    Serial1.println(ch.dbg.isr_cnt);
    Serial1.println(ch.dbg.lock_cnt); 
    Serial1.println("panic ");
    Serial1.println(ch.dbg.panic_msg);
    Serial1.println("ctx switch ");
    Serial1.println(ch.kernel_stats.n_ctxswc);
  }
}
//------------------------------------------------------------------------------
// continue setup() after chBegin().
void chSetup() {
  // Start threads.
  chThdCreateStatic(waThreadController, sizeof(waThreadController),
    NORMALPRIO + 3, ThreadController, NULL);
  chThdCreateStatic(waThreadControllerPublisher, sizeof(waThreadControllerPublisher),
    NORMALPRIO + 2, ThreadControllerPublisher, NULL);
  chThdCreateStatic(waThreadCanRead, sizeof(waThreadCanRead),
    NORMALPRIO + 1, ThreadCanRead, NULL);
  chThdCreateStatic(waThreadCanRequest, sizeof(waThreadCanRequest),
    NORMALPRIO + 1, ThreadCanRequest, NULL);
  chThdCreateStatic(waThreadRosService, sizeof(waThreadRosService),
    NORMALPRIO + 1, ThreadRosService, NULL);
  chThdCreateStatic(waThreadKernelReport, sizeof(waThreadKernelReport),
    NORMALPRIO + 1, ThreadKernelReport, NULL);
}
//------------------------------------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial1.begin(115200);
  Serial1.println("Startup initiated!");
  setupOdrvCan();
  nh.getHardware()->setBaud(115200);
  Serial1.println(nh.getHardware()->getBaud());
  nh.initNode();
  nh.subscribe(alive_led);
  for (uint8_t i=0; i<NUM_DRIVES; i++){
    axis[i].init();
  }
  controller.init();
  
  // Initialize OS and then call chSetup.
  chBegin(chSetup);
  // chBegin() resets stacks and should never return.
  while (true) {}
}
//------------------------------------------------------------------------------
// loop() is the main thread.  Not used in this example.
void loop() {
}
