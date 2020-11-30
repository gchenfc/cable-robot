# Notes

## Nov 30, 2020

Odrive (the one closer to the desk) is running on "latest" firmware.  Note that there are quite a few changes from the old firmware.  For example, many constants changed due to the units being rotations instead of counts.

Rewrote Teensy code to be just a barebones Serial <-> CAN transceiver.  Communicates Serial over ttyUSB0 aka Serial1.

### Setup

1. Plug in power supply.
2. Open up odrivetool at ttyACM# in terminal (note, due to EMI, the connection will cut out once motor commands are issued.  Unplug and replug and/or exit and re-enter odrivetool to reconnect)
3. Plug in Teensy.
4. Open up serial monitor.  (TODO(gerry): make python interface with pyserial or something)

### Serial Usage

`<axis#>n<cmd#>c<arg0>,<arg1>,...,<argN>`

Examples:

* `0n11c1,1` - set controller mode to 1 (torque control) and input mode to 1 (passthrough aka normal) on axis 0
* `0n7c8` - set requested mode on axis 0 to mode 8 (closed loop control)
* `1n14c0.3` - set torque command on axis 1 to 0.3Nm (note, must be in mode 1, ie `1n11c1,1` to take effect)
* `1n12c10.0,0,0` - set position command on axis 1 to 10.0 rev, with 0 ff vel and 0 ff torque (must be in mode 3)
* `1n9c` - requests the encoder position (in rotations) from axis 1
* `0n20c` - requests the setpoint and measured Q-current from axis 0

Note that commands are in decimal instead of hex.

See [CAN specification](https://docs.odriverobotics.com/can-protocol) for more details on commands
