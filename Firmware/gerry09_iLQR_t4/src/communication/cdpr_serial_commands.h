#pragma once

// These are the first characters of the serial commands sent to the controller
namespace SerialPrefixes {
enum : char {
  CALIBRATION = 'c',
  CONTROLLER = 'g',
  SPRAY = 's',
  DEBUG = 'd',
  // CAN_PASSTHROUGH = ? // TODO(gerry)
  LEGACY_TRACKING_CONTROLLER = 't',
  SETPOINT = 'x',
  TRACKING = 'k',
};
}  // namespace SerialPrefixes

// These are the commands for the "setpoint" manager
namespace SetpointCommands {
enum : char {
  // State control
  STOP0 = '0',
  STOP1 = '6',
  STOP2 = '8',
  TRAVEL = '1',
  START = '2',
  PAUSE = '3',
  RESET = '4',
  GOTO_TIME = '5',

  // Retrieve info
  POLL_STATUS = '?',
  POLL_SPLINE_STATUS = '!',
  DEBUG_COMPUTE_AND_PRINT_TRAVEL_SPLINE_SAMPLED = '%',
  SPLINE_DEBUG_PRINT_NUM_SEGMENTS = '#',
  SPLINE_DEBUG_SAMPLE_1S = '*',  // Print spline from 0s to 1s, every 0.1s
  SPLINE_QUERY_AT_TIME = '>',    // `x>0.5` would print x, xdot at time 0.5
  SPLINE_QUERY_COEFFS = '<',     // `x<3` would print coeffs for segment 3
  WAYPOINTS_PRINT_NUM_WAYPOINTS = '@',
  WAYPOINTS_PRINT_WAYPOINT = '$',
  // WAYPOINTS_DEBUG_SAMPLE_1S // TODO!!!

  // Set parameters
  SET_SWITCH_TO_RUN_THRESHOLD =
      't',  // If we are not within this threshold to the first setpoint, don't
            // switch to "RUN" mode
  SET_TRAVEL_SPEED = 's',          // e.g. `xs0.5` sets travel speed to 0.5 m/s
  SET_TRAVEL_ACCEL = 'a',          // e.g. `xa1.5` sets travel accel to 1.5 m/s
  SET_WORKSPACE_LIMITS_REL = 'l',  // e.g. xlu0.5 sets upper lim to height-0.5
  SET_WORKSPACE_LIMITS_ABS = 'L',  // e.g. xLu0.5 sets upper lim to 0.5 (udlrcw)
  SET_WAYPOINT_SPEED = 'S',
  SET_WAYPOINT_ACCEL = 'A',

  // Spline editing
  SPLINE_RESET = '-',
  SPLINE_ADD_SEGMENT = '+',

  // Waypoint
  WAYPOINT_SET = 'w',
  WAYPOINT_ADD = 'n',
  WAYPOINT_CLEAR = 'c',
};
}  // namespace SetpointCommands

// These are the commands for the "tracking" controller manager
namespace TrackingCommands {
enum : char {
  IDLE = '0',
  GRAVITY_COMP = '1',
  POSITION_CONTROL = '2',

  POLL_STATUS = '?',

  SET_HOLDING_TORQUE = 'h',
  SET_MAX_DISTANCE_TO_SETPOINT = 'd',
  SET_MAX_VEL_SETPOINT = 'v',
  SET_MAX_ACC_SETPOINT = 'a',

  SET_GAINS = 'K',
};
}  // namespace TrackingCommands
