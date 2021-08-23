/**
 * This file defines the CAN command id's used by the odrive.  Please refer to
 * https://docs.odriverobotics.com/can-protocol for more details.
 */

#pragma once

#include <string>

enum {
  MSG_CO_NMT_CTRL = 0x000,       // 0  // CANOpen NMT Message REC
  MSG_ODRIVE_HEARTBEAT,          // 1
  MSG_ODRIVE_ESTOP,              // 2
  MSG_GET_MOTOR_ERROR,           // 3  // Errors
  MSG_GET_ENCODER_ERROR,         // 4
  MSG_GET_SENSORLESS_ERROR,      // 5
  MSG_SET_AXIS_NODE_ID,          // 6
  MSG_SET_AXIS_REQUESTED_STATE,  // 7
  MSG_SET_AXIS_STARTUP_CONFIG,   // 8
  MSG_GET_ENCODER_ESTIMATES,     // 9
  MSG_GET_ENCODER_COUNT,         // 10
  MSG_SET_CONTROLLER_MODES,      // 11
  MSG_SET_INPUT_POS,             // 12
  MSG_SET_INPUT_VEL,             // 13
  MSG_SET_INPUT_TORQUE,          // 14
  MSG_SET_VEL_LIMIT,             // 15
  MSG_START_ANTICOGGING,         // 16
  MSG_SET_TRAJ_VEL_LIMIT,        // 17
  MSG_SET_TRAJ_ACCEL_LIMITS,     // 18
  MSG_SET_TRAJ_INERTIA,          // 19
  MSG_GET_IQ,                    // 20
  MSG_GET_SENSORLESS_ESTIMATES,  // 21
  MSG_RESET_ODRIVE,              // 22
  MSG_GET_VBUS_VOLTAGE,          // 23
  MSG_CLEAR_ERRORS,              // 24
  MSG_CO_HEARTBEAT_CMD = 0x700,  // ??  // CANOpen NMT Heartbeat  SEND
};

static constexpr char const* COMMANDS[] = {
    "MSG_CO_NMT_CTRL",               // 0  // CANOpen NMT Message REC
    "MSG_ODRIVE_HEARTBEAT",          // 1
    "MSG_ODRIVE_ESTOP",              // 2
    "MSG_GET_MOTOR_ERROR",           // 3  // Errors
    "MSG_GET_ENCODER_ERROR",         // 4
    "MSG_GET_SENSORLESS_ERROR",      // 5
    "MSG_SET_AXIS_NODE_ID",          // 6
    "MSG_SET_AXIS_REQUESTED_STATE",  // 7
    "MSG_SET_AXIS_STARTUP_CONFIG",   // 8
    "MSG_GET_ENCODER_ESTIMATES",     // 9
    "MSG_GET_ENCODER_COUNT",         // 10
    "MSG_SET_CONTROLLER_MODES",      // 11
    "MSG_SET_INPUT_POS",             // 12
    "MSG_SET_INPUT_VEL",             // 13
    "MSG_SET_INPUT_TORQUE",          // 14
    "MSG_SET_VEL_LIMIT",             // 15
    "MSG_START_ANTICOGGING",         // 16
    "MSG_SET_TRAJ_VEL_LIMIT",        // 17
    "MSG_SET_TRAJ_ACCEL_LIMITS",     // 18
    "MSG_SET_TRAJ_INERTIA",          // 19
    "MSG_GET_IQ",                    // 20
    "MSG_GET_SENSORLESS_ESTIMATES",  // 21
    "MSG_RESET_ODRIVE",              // 22
    "MSG_GET_VBUS_VOLTAGE",          // 23
    "MSG_CLEAR_ERRORS",              // 24
    "MSG_CO_HEARTBEAT_CMD",          // ??  // CANOpen NMT Heartbeat  SEND
};