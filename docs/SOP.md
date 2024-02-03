# SOP

--------------------------------------------------------------------------------

## Setup

### 1. Assembly
1. Bolt winch assemblies to frame

### 2. Communication
1. Secure down controller (MCU) box
2. Secure down E-STOP
3. Plug-in E-STOP
4. Run CAT-5 cable between ODrives (be thoughtful about cable management)
5. Run CAT-5 cable to controller (MCU) box (be thoughtful about cable management)
6. Plug-in controller (MCU) to laptop via micro-USB

### 3. Power
1. Plug in 2x LiPo Balance Alarms
2. Secure 48V power supply and box to frame
3. Plug in 48V Power Supply to "Box" (current-limited DC-DC converter)
4. Turn on current-limited DC-DC converter
5. Plug in series-battery to power bus (inside "box")
6. Connect power cables of 2 ODrives to splitter (be thoughtful about cable management)
7. Plug in "box" (power bus) to splitter
8. Turn battery switch to "1" -> "1 & 2" -> "2" (inside box)

### 4. Setup
1. Calibrate motors (`c4`)
2. Pull cables through pulleys and tie to plastic hooks
3. Roughly turn winches so that cables are roughly taut against pulleys
4. Set robot to "hold" mode (`g6;g8`)
5. Re-wind cables by pulling each cable to the opposite corner and allowing the cable to re-wind onto the winch in a controlled manner.
6. Zero motors (`c14`)
7. Attach cable hooks to spray can carriage
8. Dry-run (optional)

### 5. Spray Can Actuator
1. Open up the battery box (bottom of carriage).  Connect the ARDrone battery to the ESC (used for BEC) and place them back in the battery box in an orderly fashion.
2. If the battery box is not already connected to the wireless receiver box (front of carriage), connect it.
3. Check if the red LED on the bluetooth module is lit up.  If not, unplug it and replug it.
   * Note: there should be 2 red LEDs: one for the Arduino and one for the bluetooth module.
4. Place a spray can in the carriage and secure it.
5. Clip the servo onto the spray can.
6. Plug the servo into the wireless receiver box.
7. Test-paint with `s1` and `s0` (optional)

--------------------------------------------------------------------------------

## Setup - AIR Box

### 1. Assembly
1. Assemble frame / pulley mounting points
2. Place AIR box in frame

### 2. RPi ssh
* check route list (mac: netstat -nr)
* ssh pi@143.215.191.229
  * Ask Gerry to add your ssh key to the pi
* TODO: figure out how to get ethernet working, since wifi is too unreliable.

### 3. SOFTWARE
You need to clone/download 3 github repos:
1. `cable-robot` - https://github.gatech.edu/borglab/cable-robot
   1. This is for the main **cable robot control panel**, and also contains cable robot Firmware.
2. `raw-air` - https://github.com/gchenfc/RAW-AIR
   1. For the arm!!!!
3. `art_skills` - https://github.com/gchenfc/art_skills
   1. For the iPad

### 4. Startup and Calibration
1. Calibrate motors
2. Hold
3. Move to home and zero
4. Calibrate robot
5. 

--------------------------------------------------------------------------------

## Operation

### Things to periodically check
* Main battery balance
* Carriage battery/BEC temperature
* Carriage battery voltage

# AIR SOP

Construct robot as usual

## Commands to run

For all of these, remember to do these for BOTH CONTROLLERS!!! (`gs0` / `gs1`)

* Max tension: `kKM300` (300N)
* Distance threshold to stop tracking: `kd0.5` (0.5m)
* Limits: `xLl1.0;xLd0.75;xLr2.0;xLu1.3;` (left, down, right, up)
* Speed: `xs0.2;xS0.2` this is pretty fast but reasonable

CHECK TO MAKE SURE THE CABLES ARE ROUTED THROUGH THE PULLEYS CORRECTLY

## Setting up all the servers
### RPi

This is used if the controller pcb (Teensy) is connected to the RPi via USB instead of to your local computer (via USB).  In that case, you will run a server which forwards the serial port over websockets using a custom protocol.

* ssh into the RPi and run `cd ~/GIT_REPOS/cable-robot/site && python serial_server.py`
* tunnel the port, e.g. by running (from your local computer): `ssh -L 8765:localhost:8765 pi@ipaddressofpi`

Also, on your local computer, in `site/index.html`, make sure at the top you're using `serial_over_ws.js` instead of `serial.js`, i.e.  
```html
<!-- <script defer src="serial.js"></script> -->
<script defer src="serial_over_ws.js"></script>
```

Note that, for calibration, you will also probably want to run the `gerry00_calibrate.ipynb` jupyter notebook on the RPi.  You can either run `jupyter notebook` in the ssh shell, or in VSCode remote ssh extension.

### iPad
In the `art_skills` repo, run `cd whiteboard && python server.py`.  This will start up a server that listens for the iPad to connect to it and forwards it to the cable robot control panel index.html.  Also, it will print out an ip address.  For example,
```
STARTING UP!!!
Serving whiteboard at: 143.215.94.191:5900
Serving robot at: localhost:5904
Serving robot at: localhost:5906
Serving fit server at ('::1', 5902, 0, 0), ('127.0.0.1', 5902)
```
In this case, we would copy `143.215.94.191`.  Then, edit the `art_skills/whiteboard/client/main.js` file and at the very top, paste the ip address into the `HOST` variable.

On the control panel (after refresh), the debug console should show `Connected to ipad!!!` and the python should print `Robot connection opened!`.

~~

Next, launch a server to host the whiteboard/client/index.html somehow (e.g. in VSCode, install the Live Server extension and click "Go Live" in the bottom right corner).  Then, open up the whiteboard on the iPad by going to `143.215.94.191:5500/whiteboard/client` (or whatever the IP address is)  Notice the Live Share extension defaults to port number 5500 (the "Go Live" icon says the port number), but if you're using a different port/server, use the correct port number.  Also, make sure that you don't do `https://143...`.  I don't know why but this doesn't work - just type it into the address bar without the https.

On the python server, you should see `Whiteboard connection opened!`.

### Arm
In the RAW-AIR repo, run `cd RAW-AIR/arm && python arm_server.py`.  This should be run on whatever computer is currently connected to the Teensy via USB.  Currently, this may or may not work on the RPi, so you might have to:
* Edit the port to probably `/dev/ttyACM2`.  Currently, the Teensy creates 3 virtual serial ports, and the 3rd one is dedicated to the arm, while the other 2 are for the cable robot.
* Edit the HOST variable to be the RPi's ip address.  I'm not sure, but I'm pretty sure that when you host as localhost, it doesn't allow other devices to connect to the websocket even if they type the correct Teensy's ip address.  Something about the way it's broadcasted.
