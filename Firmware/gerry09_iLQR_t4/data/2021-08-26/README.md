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
