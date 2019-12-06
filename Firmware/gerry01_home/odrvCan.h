/*

Please see https://discourse.odriverobotics.com/t/can-interface-available-for-testing/1448
for the CAN protocol
also see ../CAN_firmware/Firmware/communication/can_simple.hpp

*/

#include <FlexCAN.h>

#ifndef __MK66FX1M0__
//   #error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif

enum CanMsg_t {
    MSG_CO_NMT_CTRL = 0x000,       // CANOpen NMT Message REC
    MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
    MSG_ODRIVE_HEARTBEAT = 0x001,
    MSG_ODRIVE_ESTOP,               // send ESTOP
    MSG_GET_MOTOR_ERROR,            // Read Errors
    MSG_GET_ENCODER_ERROR,
    MSG_GET_SENSORLESS_ERROR,
    MSG_SET_AXIS_NODE_ID,           // set CAN id
    MSG_SET_AXIS_REQUESTED_STATE,   // set state
    MSG_SET_AXIS_STARTUP_CONFIG,
    MSG_GET_ENCODER_ESTIMATES,      // get position
    MSG_GET_ENCODER_COUNT,          // get position
    MSG_MOVE_TO_POS,
    MSG_SET_POS_SETPOINT,
    MSG_SET_VEL_SETPOINT,
    MSG_SET_CUR_SETPOINT,
    MSG_SET_VEL_LIMIT,
    MSG_START_ANTICOGGING,
    MSG_SET_TRAJ_VEL_LIMIT,
    MSG_SET_TRAJ_ACCEL_LIMITS,
    MSG_SET_TRAJ_A_PER_CSS,
    MSG_GET_IQ,
    MSG_GET_SENSORLESS_ESTIMATES,
    MSG_RESET_ODRIVE,
    MSG_GET_VBUS_VOLTAGE,
    MSG_SET_PI_GAIN,
    //==================customized item===============
    MSG_SET_POSITION_GAIN,
    MSG_CLEAR_ERROR,
    MSG_GET_CURRENT_LIMIT,
    MSG_SET_CURRENT_LIMIT,
    MSG_SET_LINEAR_COUNT
    //==================end of customized item===============
};
enum Error_t {
    ERROR_NONE = 0x00,
    ERROR_INVALID_STATE = 0x01, //<! an invalid state was requested
    ERROR_DC_BUS_UNDER_VOLTAGE = 0x02,
    ERROR_DC_BUS_OVER_VOLTAGE = 0x04,
    ERROR_CURRENT_MEASUREMENT_TIMEOUT = 0x08,
    ERROR_BRAKE_RESISTOR_DISARMED = 0x10, //<! the brake resistor was unexpectedly disarmed
    ERROR_MOTOR_DISARMED = 0x20, //<! the motor was unexpectedly disarmed
    ERROR_MOTOR_FAILED = 0x40, // Go to motor.hpp for information, check odrvX.axisX.motor.error for error value 
    ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x80,
    ERROR_ENCODER_FAILED = 0x100, // Go to encoder.hpp for information, check odrvX.axisX.encoder.error for error value
    ERROR_CONTROLLER_FAILED = 0x200,
    ERROR_POS_CTRL_DURING_SENSORLESS = 0x400,
    ERROR_WATCHDOG_TIMER_EXPIRED = 0x800,
    ERROR_ESTOP_REQUESTED = 0x1000
};

enum State_t {
    AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
    AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
    AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
    AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
    AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8,  //<! run closed loop control
    AXIS_STATE_LOCKIN_SPIN = 9,       //<! run lockin spin
    AXIS_STATE_ENCODER_DIR_FIND = 10,
};

static uint8_t hex[17] = "0123456789abcdef";

// -------------------------------------------------------------
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}
static void msgDump(uint8_t *bytePtr) {
  Serial.print(*(int32_t*)bytePtr);
  Serial.print('\t');
  Serial.print(*(int32_t*)(bytePtr+4));
  Serial.print('\t');
  Serial.print(*(float*)bytePtr);
  Serial.print('\t');
  Serial.println(*(float*)(bytePtr+4));
}

// -------------------------------------------------------------
void setupOdrvCan(void)
{
  Can0.begin(500000);
  //if using enable pins on a transceiver they need to be set on
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  Can0.setNumTXBoxes(5);
}

// // -------------------------------------------------------------
// CAN_message_t inMsg;
// void updateOdrvCan(void)
// {
//     while (Can0.available()) 
//     {
//         Can0.read(inMsg);
//     //   msgDump(inMsg.buf);
//     }
//     requestInfo(0, 0x00A);
//     requestInfo(0, 0x017);
//     delay(10);
// }

uint8_t requestInfo(uint16_t nodeID, uint8_t cmdId) {
  CAN_message_t outMsg;
  outMsg.id = cmdId | (nodeID << 5);
  outMsg.rtr = 1;
  outMsg.len = 8;
  outMsg.ext = 0;
  outMsg.buf[0] = 0;
  outMsg.buf[1] = 0;
  outMsg.buf[2] = 0;
  outMsg.buf[3] = 0;
  outMsg.buf[4] = 0;
  outMsg.buf[5] = 0;
  outMsg.buf[6] = 0;
  outMsg.buf[7] = 0;
  uint8_t success = Can0.write(outMsg);
  return success;
}

uint8_t sendMsg(uint16_t nodeID, uint16_t cmdId, int32_t data1, int32_t data2) {
  CAN_message_t outMsg;
  outMsg.id = cmdId | (nodeID << 5);
  outMsg.rtr = 0;
  outMsg.len = 8;
  outMsg.ext = 0;
  outMsg.buf[0] = (data1 >> 0) & 0xFF;
  outMsg.buf[1] = (data1 >> 8) & 0xFF;
  outMsg.buf[2] = (data1 >> 16) & 0xFF;
  outMsg.buf[3] = (data1 >> 24) & 0xFF;
  outMsg.buf[4] = (data2 >> 0) & 0xFF;
  outMsg.buf[5] = (data2 >> 8) & 0xFF;
  outMsg.buf[6] = (data2 >> 16) & 0xFF;
  outMsg.buf[7] = (data2 >> 24) & 0xFF;
  uint8_t success = Can0.write(outMsg);
  return success;
}
uint8_t sendMsg(uint16_t nodeID, uint16_t cmdId, int32_t data1) {
  return sendMsg(nodeID, cmdId, data1, 0);
}
uint8_t sendMsg(uint16_t nodeID, uint16_t cmdId, int32_t data1, int16_t data2, int16_t data3) {
  int32_t dataCombo = ((int32_t)data3)<<16 | ((int32_t)data2);
  return sendMsg(nodeID, cmdId, data1, dataCombo);
}
uint8_t sendMsg(uint16_t nodeID, uint16_t cmdId, float data1, float data2) {
  int32_t buf1, buf2;
  memcpy(&buf1, &data1, sizeof(data1));
  memcpy(&buf2, &data2, sizeof(data2));
  return sendMsg(nodeID, cmdId, buf1, buf2);
}
uint8_t sendMsg(uint16_t nodeID, uint16_t cmdId, float data) {
  return sendMsg(nodeID, cmdId, data, 0);
}