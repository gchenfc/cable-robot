To run unit tests, do:

```bash
mkdir build
cd build
cmake ..
make check
```

Running unit tests will not build the Arduino project - please compile/upload using Teensyduino.

## Annoying Quirks

### Includes

For including with relative paths, Arduino uses the path relative to the file instead of relative to the project root.

### Unit tests
Arduino isn't smart enough to not compile unrelated cpp files so you need to enclose unit tests intended to be run on the computer with
```c++
#ifndef ARDUINO
...

#endif
```

# Command Set

If in doubt, check `communication/debug.h`.

Multiple commands can be sent at a time by separating them by semicolons.

Typical workflow:

* Calibrate
  * `c4`
* Zero
  * TODO: document zeroing procedure
* Running:
  * `g6;g8`
  * move end effector to approximate starting position
  * `g1`
  * `g2`

## Calibration-related messages (`c` commands)

format: `c#`

In general, the tens-digit describes the type of command, and the ones digit denotes which motor (4 denotes all-motors).  E.g. `c2` means calibrate motor 2 (zero-indexed), and `c4` means calibrate all the motors.

command | description
|-------| -----------
`c0` - `c3` | run calibration on a single winch motor
`c4`        | run calibration on all the winch motors
`c10` - `c13` | zero a single winch motor
`c14`       | zero all the winches
`c15`       | print out the zeros
`c20` - `c23` | load zero from EEPROM for a single motor (this should automatically happen on startup)
`c24`       | load all zeros
`c30,#` - `c33,#` | set the winch zero such that the length equals the number given
`c34,#,#,#,#` | set the winch zeros such that the lengths equal the numbers given
`c40,#,#,#` - `c43,#,#,#` | set the winch length correction parameters
`c44,#,#,#,#,#,#,#,#,#,#,#,#` | set all the winch length correction parameters
`c50,#,#` - `c53,#,#` | set the pulley mount point
`c54,#,#,#,#,#,#,#,#` | set all the pulley mount points

## High-level control messages (`g` commands)

format: `g#`

command | description
|-------| -----------
`g0`    | clear errors
`g1`    | go to starting position of trajectory
`g2`    | start running trajectory
`g3`    | stop running trajectory (experimental)
`g4`    | reset to beginning of trajectory
`g5`    | set to traj index (experimental)
`g6`    | hold - set all motors to 0.2Nm which effectively loosely holds the end effector in position
`g7`    | release - set all motors to 0.0Nm which drops the end effector
`g8`    | closed loop control - enables the motors
`g10,#` | Set the "hold torque" to this amount (in Nm).  Default is 0.2Nm.
`gs#`   | if using a switchable controller, switch to controller #

## Tracking Control (`t` commands)

format: `t_#`

command | description
|-------| -----------
`ta#,#` | go to the absolute coordinate #, # in meters
`tr#`   | go to the right by # meters
`tl#`   | go to the left by # meters
`tu#`   | go to up by # meters
`td#`   | go to down by # meters
`ts#`   | set movement speed to # m/s
`tA#`   | set movement acceleration to # m/s^2
`tLu#`  | set upper limit to frame height - #
`tLd#`  | set lower limit to #
`tLl#`  | set left limit to #
`tLr#`  | set right limit to frame width - #

## Parameter adjustment (`K` commands)

### Gouttefarde controller:

format: `K_#`

command | description
|-------| -----------
`Kp#`   | Set proportional gain
`Ki#`   | Set integral gain
`Kd#`   | Set derivative gain
`Kv#`   | Set viscous friction coefficient (fv)
`Ks#`   | Set static friction coefficient (fs)
`Ku#`   | Set static friction parameter (mu)
`Km#`   | Set "midpoint" tension in Newtons!!! (t_m)

(friction force) = fv * (cable vel) + fs * tanh(mu * (cable vel))

## Spray-paint messages (`s` commands)

format: `s#`

command | description
|-------| -----------
`s0`    | spray off
`s1`    | spray on

## CAN passthrough (`_n_c` commands)
Use this to send a CAN message directly to an odrive if you want some "custom" functionality.

format: `#n#c`
format: `#n#c#`
format: `#n#c#,#`
format: `#n#c#,#,#`

First number is the winch number, second number is the CMD ID (message id).  The numbers after the `c` denote the arguments, and each message has a different number of arguments.

See [ODrive CAN docs](https://betadocs.odriverobotics.com/can-protocol) (link may be broken, just google "odrive CAN protocol").

CAN table reproduced here:

CMD ID | Name | Sender | Signals | Start byte | Signal Type | Bits | Factor | Offset
--:    | :--  | :--  | :-- | :-- | :-- | :-- | :-- | :--
0x000 (0) | CANOpen NMT Message\*\* | Master | - | - | - | - | - | -
0x001 (1) | ODrive Heartbeat Message | Axis | Axis Error<br>Axis Current State<br>Controller Status | 0<br>4<br>7 | Unsigned Int<br>Unsigned Int<br>Bitfield | 32<br>8<br>8 | -<br>-<br>- | -<br>-<br>-
0x002 (2) | ODrive Estop Message | Master | - | - | - | - | - | -
0x003 (3) | Get Motor Error\* | Axis  | Motor Error | 0 | Unsigned Int | 64 | 1 | 0
0x004 (4) | Get Encoder Error\*  | Axis | Encoder Error | 0 | Unsigned Int | 32 | 1 | 0
0x005 (5) | Get Sensorless Error\* | Axis | Sensorless Error | 0 | Unsigned Int | 32 | 1 | 0
0x006 (6) | Set Axis Node ID | Master | Axis CAN Node ID | 0 | Unsigned Int | 32 | 1 | 0
0x007 (7) | Set Axis Requested State | Master | Axis Requested State | 0 | Unsigned Int | 32 | 1 | 0
0x008 (8) | Set Axis Startup Config | Master | - Not yet implemented - | - | - | - | - | -
0x009 (9) | Get Encoder Estimates\* | Master | Encoder Pos Estimate<br>Encoder Vel Estimate | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x00A (10) | Get Encoder Count\* | Master | Encoder Shadow Count<br>Encoder Count in CPR | 0<br>4 | Signed Int<br>Signed Int | 32<br>32 | 1<br>1 | 0<br>0
0x00B (11) | Set Controller Modes | Master | Control Mode<br>Input Mode | 0<br>4 | Signed Int<br>Signed Int | 32<br>32 | 1<br>1 | 0<br>0
0x00C (12) | Set Input Pos | Master | Input Pos<br>Vel FF<br>Torque FF | 0<br>4<br>6 | IEEE 754 Float<br>Signed Int<br>Signed Int | 32<br>16<br>16 | 1<br>0.001<br>0.001 | 0<br>0<br>0
0x00D (13) | Set Input Vel | Master | Input Vel<br>Torque FF | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x00E (14) | Set Input Torque | Master | Input Torque | 0 |  IEEE 754 Float | 32 | 1 | 0
0x00F (15) | Set Limits | Master | Velocity Limit<br>Current Limit | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br> | 1<br>1 | 0<br>0
0x010 (16) | Start Anticogging | Master | - | - | - | - | - | -
0x011 (17) | Set Traj Vel Limit | Master | Traj Vel Limit | 0 | IEEE 754 Float | 32 | 1 | 0
0x012 (18) | Set Traj Accel Limits | Master | Traj Accel Limit<br>Traj Decel Limit | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x013 (19) | Set Traj Inertia | Master | Traj Inertia | 0 | IEEE 754 Float | 32 | 1 | 0
0x014 (20) | Get IQ\* | Axis | Iq Setpoint<br>Iq Measured | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x015 (21) | Get Sensorless Estimates\* | Master | Sensorless Pos Estimate<br>Sensorless Vel Estimate | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x016 (22) | Reboot ODrive | Master\*\*\* | - | - | - | - | - | -
0x017 (23) | Get Vbus Voltage | Master\*\*\* | Vbus Voltage | 0 | IEEE 754 Float | 32 | 1 | 0
0x018 (24) | Clear Errors | Master | - | - | - | - | - | -
0x019 (25) | Set Linear Count | Master | Position | 0 | Signed Int | 32 | 1 | 0
0x01A (26) | Set Position Gain | Master | Pos Gain | 0 | IEEE 754 Float | 32 | 1 | 0
0x01B (27) | Set Vel Gains | Master | Vel Gain<br>Vel Integrator Gain | 0<br>4 | IEEE 754 Float<br>IEEE 754 Float | 32<br>32 | 1<br>1 | 0<br>0
0x700 (1792) | CANOpen Heartbeat Message\*\* | Slave | - | -  | - | - | - | -
-|-|-|----------------------------------|-|--------------------|-|-|-

All multibyte values are little endian (aka Intel format, aka least significant byte first).



# Anticogging

Connect to the odrive over USB.  You probably want to use a couple extra ferrite beads to make sure the connection doesn't drop out.

do:
```
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.config.vel_integrator_gain = 5
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.start_anticogging_calibration()
```
Also try setting vel_gain larger (e.g. 0.5) if it's having trouble.

Check the status with `odrv0.axis0.controller.config.anticogging.index`.
Once it's done, `odrv0.axis0.controller.anticogging_valid` should be true.  Try setting torque to 0 and spin it around to feel.

Quick-paste:
```
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```
wait
```
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.config.vel_integrator_gain = 5
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.start_anticogging_calibration()
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis1.controller.config.vel_integrator_gain = 5
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.controller.start_anticogging_calibration()
```

For reference, here's what the full config looks like (although most details are probably flexible, this is just a reference):
```
anticogging:
  anticogging_enabled: True (bool)
  calib_anticogging: False (bool)
  calib_pos_threshold: 1.0 (float)
  calib_vel_threshold: 1.0 (float)
  cogging_ratio: 1.0 (float)
  index: 0 (uint32)
  pre_calibrated: False (bool)
axis_to_mirror: 255 (uint8)
circular_setpoint_range: 1.0 (float)
circular_setpoints: False (bool)
control_mode: 3 (uint8)
electrical_power_bandwidth: 20.0 (float)
enable_current_mode_vel_limit: False (bool)
enable_gain_scheduling: False (bool)
enable_overspeed_error: True (bool)
enable_vel_limit: True (bool)
gain_scheduling_width: 10.0 (float)
homing_speed: 0.25 (float)
inertia: 0.0 (float)
input_filter_bandwidth: 2.0 (float)
input_mode: 1 (uint8)
load_encoder_axis: 1 (uint8)
mechanical_power_bandwidth: 20.0 (float)
mirror_ratio: 1.0 (float)
pos_gain: 100.0 (float)
spinout_electrical_power_threshold: 10.0 (float)
spinout_mechanical_power_threshold: -10.0 (float)
steps_per_circular_range: 1024 (int32)
torque_mirror_ratio: 0.0 (float)
torque_ramp_rate: 0.009999999776482582 (float)
vel_gain: 0.20000000298023224 (float)
vel_integrator_gain: 5.0 (float)
vel_limit: 5.0 (float)
vel_limit_tolerance: 10.0 (float)
vel_ramp_rate: 1.0 (float)
```