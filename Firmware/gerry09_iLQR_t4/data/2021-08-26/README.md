Naming convention:

circleX_gainX_offX_lowX.txt

* circleX refers to 10s per circle (1) vs 5s per circle (2)
* gainX refers to the proportional gain (see code) either 5, 25, or 75
* offX refers to the "median offset" which is the torque that will be applied when error=0, divided by 10.  e.g. off7 => 0.7 Nm
* lowX refers to the minimum torque allowed.
  * 02 => 0.2Nm
  * -01 => -0.1Nm
  * -01_2 => -0.1Nm (same as previous, just ran it again because it seemed flukily good)
  * 0 => 0Nm
  * I also tried -.2Nm but didn't save it because it was bad and crashed I think.

# ATL

Using the setting -0.1 to 1.2 Nm

ATL_controller_1e2_1mps.txt has additional annotations:
```
S   Nodes 0 and 1:
S   system: Error(s):
S     ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE
S   axis0
S     axis: Error(s):
S       AXIS_ERROR_WATCHDOG_TIMER_EXPIRED
S       AXIS_ERROR_ESTOP_REQUESTED
S     motor: Error(s):
S       MOTOR_ERROR_SYSTEM_LEVEL
S       MOTOR_ERROR_UNKNOWN_TORQUE
S     sensorless_estimator: no error
S     encoder: no error
S     controller: Error(s):
S       CONTROLLER_ERROR_SPINOUT_DETECTED
S   axis1
S     axis: Error(s):
S       AXIS_ERROR_ESTOP_REQUESTED
S     motor: Error(s):
S       MOTOR_ERROR_SYSTEM_LEVEL
S     sensorless_estimator: no error
S     encoder: no error
S     controller: no error
S
S
S   system: no error
S   axis2
S     axis: Error(s):
S       AXIS_ERROR_WATCHDOG_TIMER_EXPIRED
S       AXIS_ERROR_ESTOP_REQUESTED
S     motor: no error
S     sensorless_estimator: no error
S     encoder: no error
S     controller: no error
S   axis3
S     axis: Error(s):
S       AXIS_ERROR_WATCHDOG_TIMER_EXPIRED
S       AXIS_ERROR_ESTOP_REQUESTED
S     motor: no error
S     sensorless_estimator: no error
S     encoder: no error
S     controller: no error
```
By the way, the undervoltage trip level for axes 0/1 was set at 8V lol
