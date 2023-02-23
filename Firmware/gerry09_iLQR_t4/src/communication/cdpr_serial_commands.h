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
  STOP0 = '0',
  STOP1 = '6',
  STOP2 = '8',
  TRAVEL = '1',
  START = '2',
  PAUSE = '3',
  RESET = '4',
  GOTO_TIME = '5',

  POLL_STATUS = '?',
  DEBUG_COMPUTE_AND_PRINT_TRAVEL_SPLINE_SAMPLED = '%',

  SET_WORKSPACE_LIMITS_REL = 'l',  // e.g. xLu0.5 sets upper lim to 0.5 (udlrcw)
  SET_WORKSPACE_LIMITS_ABS = 'L',  // e.g. xLu0.5 sets upper lim to height-0.5
};

// These are the commands for the "tracking" controller manager
enum TrackingCommands {};
