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

command | description
|-------| -----------
`c0` - `c3` | run calibration on a single winch motor
`c4`        | run calibration on all the winch motors
`c10` - `c13` | zero a single winch motor
`c14`       | zero all the winches
`c15`       | print out the zeros
`c20` - `c23` | load zero from EEPROM for a single motor (this should automatically happen on startup)
`c24`       | load all zeros

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

