- [Odrive CAN](#odrive-can)
- [Teensy Commands](#teensy-commands)
  - [gerry08](#gerry08)

# Odrive CAN

[Odrive protocol](https://docs.odriverobotics.com/can-protocol)

* -n7c- : set state [enum vals](https://docs.odriverobotics.com/api/odrive.axis.axisstate)
  * 1 - idle
  * 2 - startup calibration
  * 3 - full calibration
  * 8 - closed loop
* -n11c-,- : set controller mode
  * First arg: control mode [enum vals](https://docs.odriverobotics.com/api/odrive.controller.controlmode)
    * 0 - voltage
    * 1 - torque
    * 2 - velocity
    * 3 - position
  * Second arg: input mode [enum vals](https://docs.odriverobotics.com/api/odrive.controller.inputmode)
    * 1 - passthrough (normal)
    * 2 - velocity ramp
    * 3 - position filter
    * etc.
* -n12c-,-,- : position command (pos, velFF, torqueFF)
* -n13c-,- : velocity command (vel, torqueFF)
* -n14c- : torque command (torque)

# Teensy Commands

## gerry08

Setup stuff:

* -n75c : set motors to 0.2Nm of torque
* -n80c : set this as zero point

Trajectory stuff:

* -n56c : step trajectory
* -n57c : start trajectory
* -n58c : stop trajectory (and resets counter to beginning)
* -n59c- : manually set trajectory counter index
* -n60c : resume trajectory
* -n61c : pause trajectory
* -n100c- : manually activate (1) or deactivate (0) paint