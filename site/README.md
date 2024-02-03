For a better readme, please see the one in [Firmware/gerry09_iLQR_t4/README.md](../Firmware/gerry09_iLQR_t4/README.md).

## AIR notes
* Turn down speed to ts0.04
* Turn down speed in cdpr.js to 0.04
* Check anticogging calibration - Motors 0/1 seemed to be ok but Motors 2/3 did not appear to be ok.
* Set L/R/U/D limits in cable robot manually via serial
  * ```xLu1.7;xLr5;xLl0.8;xLd0.3```
* Set custom gains

* Mullion depth is 5.5"

### Serial over websockets
If you have a dedicated computer connected to the cable robot, then you can stream the serial data over a websocket and control the robot from your laptop.  To do so,

On the computer connected to the robot, run the following command:
```bash
python serial_server.py
```

On your laptop, run the following command:
```bash
ssh -L 8765:localhost:8765 <username>@<robot_ip>
```
and keep the window open (it should open to a command prompt on the robot computer).
Now, if it's not already, switch `index.html` to use 
```html
<!-- <script defer src="serial.js"></script> -->
<script defer src="serial_over_ws.js"></script>
```
to use the Serial over WebSockets version of the serial interface javascript file.

Now you can open the site as usualy by opening the `index.html` file in your browser.  You should be able to control the robot from your laptop.
