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
#include "ch.h"
#include "ds.h"
#include "util.h"
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
#include "rosSystem.h"
RosAxis axis[NUM_DRIVES] = {RosAxis(0), RosAxis(1), RosAxis(2), RosAxis(3)};
#include "CableController.h"
//------------

//Queue msg_queue[NUM_DRIVES] = {Queue(), Queue(), Queue(), Queue()};
/*
PoolObject_t PoolObject[MB_COUNT];
const size_t MB_COUNT = 6;
MEMORYPOOL_DECL(memPool, MB_COUNT, 0);
msg_t letter[MB_COUNT];
MAILBOX_DECL(mail, &letter, MB_COUNT);

static char buffers[NUM_BUFFERS][BUFFERS_SIZE];
static msg_t free_buffers_queue[NUM_BUFFERS];
static mailbox_t free_buffers;
static msg_t filled_buffers_queue[NUM_BUFFERS];
static mailbox_t filled_buffers;
 */

CableController controller;
RosSystem sysReport;

int alive_led_status = 0;
void alive_led_cb( const std_msgs::Empty& toggle_msg){
  alive_led_status ++;   // blink the led
  digitalWrite(LED_BUILTIN, alive_led_status % 2);
}
ros::Subscriber<std_msgs::Empty> alive_led("toggle_led", &alive_led_cb);

//static int inMsg_status;
//static int inMSg_dest;
//static CAN_message_t inMsg;

struct can_msg shared_msg_buffer;
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
/*
static THD_WORKING_AREA(waThreadControllerPublisher, 64);
static THD_FUNCTION(ThreadControllerPublisher, arg) {
  (void)arg;
  systime_t updateTime2 = chVTGetSystemTime();
  while (true) {
    controller.publish();
    chThdSleepMilliseconds(50);
  }
}
*/
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
  digitalWrite(13, HIGH);
  uint16_t nodeID = 0;
  CAN_message_t inMsg;
  while (true) {
    if (Can0.available()) {

      //Can0.read(inMsg);
      Can0.read(inMsg);
      nodeID = inMsg.id >> 5;
      
      if (nodeID >= NUM_DRIVES) {
        continue;
      }
      //make update of axis
      axis[nodeID].updateCAN(inMsg);
      if (shared_msg_buffer.status == 0){
        msgCopy(inMsg, &(shared_msg_buffer.msg));
        shared_msg_buffer.axis = nodeID;
        shared_msg_buffer.status = 1;
      }
      

      /*
      chMtxLock(&(msg_queue[nodeID].lock));
      Queue_Node* qn = new Queue_Node();
      qn->item = to_add;
      qn->next = NULL;
      if (msg_queue[nodeID].front == NULL) {
        msg_queue[nodeID].front = qn;
        msg_queue[nodeID].end = qn;
      }else{
        (msg_queue[nodeID].end)->next = (void*) qn;
        msg_queue[nodeID].end = (void*) qn;
      }
      chMtxUnlock(&(msg_queue[nodeID].lock));
      */
    }

    resendMsgs(); // service manual TX buffer
    chThdSleepMicroseconds(200);
  }
}
//------------------------------------------------------------------------------
// Thread RosService, ros spin thread
static THD_WORKING_AREA(waThreadRosService, 256);
static THD_FUNCTION(ThreadRosService, arg) {
  (void)arg;
  digitalWrite(13, LOW);
  uint16_t nodeID=0;
  CAN_message_t* msg2send;

  while (true) {
    nh.spinOnce();
    // chThdSleepMilliseconds(1);
    chThdSleepMicroseconds(200);
    controller.publish();
    chThdSleepMilliseconds(50);
    sysReport.publish();
    chThdSleepMilliseconds(50);

    //read from msg queue
    if (shared_msg_buffer.status == 1){
      axis[shared_msg_buffer.axis].publishCAN(shared_msg_buffer.msg);
      shared_msg_buffer.status = 0;
    }
    
    
    /*
    //chMtxLock(&(msg_queue[nodeID].lock));
      if (msg_queue[nodeID].front != NULL){
          msg2send = (CAN_message_t*) msg_queue[nodeID].front->item;
          Queue_Node* tmp = msg_queue[nodeID].front;
          msg_queue->front = (Queue_Node*) tmp -> next;
          if (tmp -> next == NULL) {
            msg_queue[nodeID].end = NULL;
          }
          axis[nodeID].readCAN(*msg2send);
          delete tmp->item;
          delete tmp;
      }
    //chMtxUnlock(&(msg_queue[nodeID].lock));
    */

    nodeID = (nodeID+1)%NUM_DRIVES;
      

    chThdSleepMilliseconds(50);

  }

}

//------------------------------------------------------------------------------
// continue setup() after chBegin().
void chSetup() {
  // Start threads.
  /*for (uint16_t i = 0; i < NUM_DRIVES; i++){
    chMtxObjectInit(&(msg_queue[i].lock));
  }*/
  
  chThdCreateStatic(waThreadController, sizeof(waThreadController),
    NORMALPRIO + 3, ThreadController, NULL);
  chThdCreateStatic(waThreadCanRead, sizeof(waThreadCanRead),
    NORMALPRIO + 3, ThreadCanRead, NULL);
  chThdCreateStatic(waThreadCanRequest, sizeof(waThreadCanRequest),
    NORMALPRIO + 2, ThreadCanRequest, NULL);
  chThdCreateStatic(waThreadRosService, sizeof(waThreadRosService),
    NORMALPRIO + 4, ThreadRosService, NULL);
}
//------------------------------------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(13, HIGH);
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
