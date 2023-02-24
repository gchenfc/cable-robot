#pragma once

// These are the first characters of the serial commands sent to the controller
enum SerialPrefixes : char {
  CALIBRATION = 'c',
  CONTROLLER = 'g',
  SPRAY = 's',
  DEBUG = 'd',
  // CAN_PASSTHROUGH = ? // TODO(gerry)
  LEGACY_TRACKING_CONTROLLER = 't',
  SETPOINT = 'x',
  TRACKING = 'k'
};

// These are the commands for the "setpoint" manager
enum SetpointCommands : char {
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
  POLL_SPLINE_STATUS = 's',
  DEBUG_COMPUTE_AND_PRINT_TRAVEL_SPLINE_SAMPLED = '%',
  SPLINE_DEBUG_PRINT_NUM_SEGMENTS = '#',
  SPLINE_DEBUG_SAMPLE_1S = '*',  // Print spline from 0s to 1s, every 0.1s
  SPLINE_QUERY_AT_TIME = '>',    // `x>0.5` would print x, xdot at time 0.5
  SPLINE_QUERY_COEFFS = '<',     // `x<3` would print coeffs for segment 3

  // Set parameters
  SET_WORKSPACE_LIMITS_REL = 'l',  // e.g. xLu0.5 sets upper lim to 0.5 (udlrcw)
  SET_WORKSPACE_LIMITS_ABS = 'L',  // e.g. xLu0.5 sets upper lim to height-0.5

  // Spline editing
  SPLINE_RESET = '-',
  SPLINE_ADD_SEGMENT = '+',
};

// These are the commands for the "tracking" controller manager
enum TrackingCommands {};
