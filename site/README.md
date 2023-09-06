For a better readme, please see the one in [Firmware/gerry09_iLQR_t4/README.md](../Firmware/gerry09_iLQR_t4/README.md).

## AIR notes
* Turn down speed to ts0.04
* Turn down speed in cdpr.js to 0.04
* Check anticogging calibration - Motors 0/1 seemed to be ok but Motors 2/3 did not appear to be ok.
* Set L/R/U/D limits in cable robot manually via serial
  * ```xLu1.7;xLr5;xLl0.8;xLd0.3```
* Set custom gains

* Mullion depth is 5.5"
