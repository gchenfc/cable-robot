/**
 * This file defines the CAN command id's used by the odrive.  Please refer to
 * https://docs.odriverobotics.com/can-protocol for more details.
 */

#pragma once

#include <string>

enum CanMsgCmdId {
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
  MSG_SET_LINEAR_COUNT,          // 25
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
    "MSG_SET_LINEAR_COUNT",          // 25
    "MSG_CO_HEARTBEAT_CMD",          // ??  // CANOpen NMT Heartbeat  SEND
};

// Traits for Can CmdId's.  Example usage:
//    auto values = CanUtils.parse<typename CanCmdTraits<CMD_ID>>(data);
// where CMD_ID is a template parameter.
template <CanMsgCmdId ID>
struct CanCmdTraits {
  using Data = void;
  using RecvCallback = std::function<void(Odrive*)>;
};

using CanUIntPair = std::pair<uint32_t, uint32_t>;
using CanIntPair = std::pair<int32_t, int32_t>;
using CanFloatPair = std::pair<float, float>;

#define MAKE_TRAITS(CMDID, DATA) \
  template <>                    \
  struct CanCmdTraits<CMDID> {   \
    using Data = DATA;           \
    using RecvCallback = std::function<void(Odrive*, DATA)>;


// MAKE_TRAITS(MSG_CO_NMT_CTRL, void)
MAKE_TRAITS(MSG_ODRIVE_HEARTBEAT, CanUIntPair)
// MAKE_TRAITS(MSG_ODRIVE_ESTOP, void)
MAKE_TRAITS(MSG_GET_MOTOR_ERROR, uint64_t)
MAKE_TRAITS(MSG_GET_ENCODER_ERROR, uint32_t)
MAKE_TRAITS(MSG_GET_SENSORLESS_ERROR, uint32_t)
MAKE_TRAITS(MSG_SET_AXIS_NODE_ID, uint32_t)
MAKE_TRAITS(MSG_SET_AXIS_REQUESTED_STATE, uint32_t)
// MAKE_TRAITS(MSG_SET_AXIS_STARTUP_CONFIG, void)
MAKE_TRAITS(MSG_GET_ENCODER_ESTIMATES, CanFloatPair)
MAKE_TRAITS(MSG_GET_ENCODER_COUNT, CanIntPair)
MAKE_TRAITS(MSG_SET_CONTROLLER_MODES, CanIntPair)
MAKE_TRAITS(MSG_SET_INPUT_POS, std::tuple<float, int16_t, int16_t>)
MAKE_TRAITS(MSG_SET_INPUT_VEL, CanFloatPair)
MAKE_TRAITS(MSG_SET_INPUT_TORQUE, float)
MAKE_TRAITS(MSG_SET_VEL_LIMIT, CanFloatPair)
// MAKE_TRAITS(MSG_START_ANTICOGGING, void)
MAKE_TRAITS(MSG_SET_TRAJ_VEL_LIMIT, float)
MAKE_TRAITS(MSG_SET_TRAJ_ACCEL_LIMITS, CanFloatPair)
MAKE_TRAITS(MSG_SET_TRAJ_INERTIA, float)
MAKE_TRAITS(MSG_GET_IQ, CanFloatPair)
MAKE_TRAITS(MSG_GET_SENSORLESS_ESTIMATES, CanFloatPair)
// MAKE_TRAITS(MSG_RESET_ODRIVE, void)
MAKE_TRAITS(MSG_GET_VBUS_VOLTAGE, CanFloatPair)
// MAKE_TRAITS(MSG_CLEAR_ERRORS, void)
MAKE_TRAITS(MSG_SET_LINEAR_COUNT, int32_t)
// MAKE_TRAITS(MSG_CO_HEARTBEAT_CMD, void)

(MSG_CO_HEARTBEAT
SS

#undef MAKET_SSS_ETTIMATES, \
    CanRAITS(MSG_CO_HEARTBEAT #undef MAKE_TRAITTSSS_ESTIMATES, Can
