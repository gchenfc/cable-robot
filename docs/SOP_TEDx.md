<style>
.title {
   text-align: center;
   margin: 30px;
}
hr {
   margin-top: 30px;
}
h1 {
   text-align: center;
}
h2 {
   /* text-align: center; */
   text-decoration: underline;
}
h3 {
   text-decoration: underline;
}
</style>

<h1 class=title> SOP for TEDxAtlanta Demo (and future demos?) </h1>

<h2 class=toc> Table of Contents </h2>

- [Preliminary](#preliminary)
  - [What configuration is this README meant for?](#what-configuration-is-this-readme-meant-for)
  - [Packing List](#packing-list)
  - [Software Installation](#software-installation)
- [General HTML Control Panel Usage](#general-html-control-panel-usage)
  - [Layout](#layout)
  - [Joystick Control](#joystick-control)
  - [Sending manual commands](#sending-manual-commands)
- [Assembly](#assembly)
  - [Strut Channel Frame](#strut-channel-frame)
  - [Electronics](#electronics)
- [Setup / Calibration](#setup--calibration)
  - [Cable Robot](#cable-robot)
  - [Arm](#arm)
  - [iPad](#ipad)
- [Operation](#operation)

--------------------------------------------------------------------------------

# Preliminary
## What configuration is this README meant for?
* Strut Channel Frame (rectangular prism of strut channel)
* AIR box
* Robot Arm brush actuator
* Laptop, connected to AIR box via USB, AIR box connects to robot arm

## Packing List
* Strut Channel Frame
  * 2x rectangles, 8'x2'
  * 4x 10' strut channel lengths
  * 2x 8' strut channel lengths (for holding backing board / paper)
  * 3x 8'x2' drywall sheets for backing board
  * 1x roll of paper (4' tall, or 6' tall if you wanna go hardcore)
  * All strut channel nuts & bolts, should be included on frames
* AIR box
  * AIR box
  * EIC power cable
  * long USB cable / isolator
  * power cable & CAT5 tether cables to robot arm
* Robot Arm brush actuator
  * Arm assembly box
* Tools
  * TODO: fill this in

## Software Installation
You need to clone/download 3 github repos:
1. `cable-robot` - https://github.gatech.edu/borglab/cable-robot
   1. This is for the main **cable robot HTML control panel**, and also contains cable robot Firmware.
   2. Mainly, you will be using the `site` folder, which contains the control panel (locally) at `site/index.html`.
2. `raw-air` - https://github.com/gchenfc/RAW-AIR
   1. This is for the arm.
   2. Mainly, you will be using the `arm/arm_server.py` file to interpret cable robot control panel commands and send them to the arm (control panel -> arm_server -> teensy -> arm)
3. `art_skills` - https://github.com/gchenfc/art_skills
   1. This is for the iPad.
   2. Mainly, you will be using the `whiteboard` folder.
   3. Also, highly recommended to install the vscode extension `Live Server` and use it to serve the `whiteboard/client` folder.

--------------------------------------------------------------------------------

# General HTML Control Panel Usage
## Layout
The HTML Control Panel shows:
* The cable robot on the top left.  Black is "estimated position", Red is "commanded position" from the control panel, and Green is drawn trajectory on the iPad.
* Joystick in the top-middle.  This is mainly just to debug in the joystick is connected or not.
* Debug messages on the top-right.
  * Top: messages sent from control panel to cable robot
  * Next: setpoint manager status.  Mostly applies to iPad.  Format: `time state status isDone`, where:
    * `time` is where in the trajectory we are in.  i.e. halfway through the trajectory.  For joystick mode, this doesn't matter, but it might matter for iPad mode.
    * `state` is 0: hold/idle, 1: painting/running, 2: traveling
    * `status` is 0: uninitialized, 2: nominal
    * `isDone` is 0: not done, 1: done (with trajectory, again, this only applies to iPad mode)
  * Next: tracking.  Format: `state`, where:
    * `state` is 0: idle, 1: "hold" aka "gravity comp", 2: "position tracking mode"
  * Bottom: waypoint.  Prints debug information about trajectories, esp waypoints follower (iPad).
* Arm Control in the center.
  * Go Home - moves the robot to the vertical up position
  * Go Storage - moves the robot to a tucked-in position then disables the motors
  * Enable/Disable - enables/disables the motors
* Low-level Serial Communication
  * `Connect`: manually connect to a serial port
    * Select the first "Triple Serial".
    * If "autoconnect" is checked, it will automatically connect and the "connect" button won't give you a list of options.
  * `Clear`: clears the messages
  * `Disconnect`: disconnects.  This is the easiest way to stop the messages so that you can stop to read what they say.
  * `Logging`: ignore these - this is for data collection but we don't need to do that.
  * `Send with \r\n`: leave this checked
  * `echo`: whether to print out the line you sent or not
  * `Auto-connect`: will auto-connect to a certain "Triple Serial" port so you don't have to click "connect" every time.  If it isn't getting the right one (messages aren't coming in), try switching the `AUTOCONNECT_INDEX` from 0/1/2 in `site/js/serial.js`.
  * `Send`: you can just hit "enter" instead.
  * Green text: this are the messages coming from the Teensy (MCU) cable robot.  Format is:
    > `[time] - 0: [est pos] - [set pos]   | [motor 0] | [motor 1] | [motor 2] | [motor 3] |   [spray]`
    * `time`: time in us, modulo 10s for readability
    * `0`: idk what this is.  I think it's robot state but it's outdated.
    * `est pos`: estimated position as theta-x-y.  I am not estimating or controlling orientation theta so it's always 0.
    * `set pos`: setpoint position ^.
    * `motor x`: format is
      > `[error code] [state] [cable length] [cable vel]`
      * `Error code` should be 0 under normal operation, 2048 if Teensy disconnected, 16384 if ESTOP pressed.
      * `State` is 1: idle (off), 8: motor on, 4/5/6/7: motor calibration sequence
      * length & vel are in SI units (m and m/s)
    * `[spray]`: ignore this.  It's outdated.

## Joystick Control
* Mode - I recommend spamming these just in case the wireless connection cuts out.
  * `A` to clear errors
  * `X` to put into `HOLD`/"gravity comp" mode
  * `Y` to put into `position tracking` mode
    * I recommend anytime *before* pressing `Y`, first press `X` (HOLD) just in case.  It's probably not necessary, but you never know.
  * `DPAD`
    * `LEFT` & `DOWN` (both!) to put into `joystick` mode (default unless iPad is connected)
    * `RIGHT` & `UP` (both!) to put into `iPad` mode (default when an iPad is connected)
  * `Press right joystick` to **ESTOP**
* Joystick
  * Left joystick controls the position setpoint when in `position tracking` mode and `joystick` mode
* Arm
  * `LB` "home" configuration
  * `LT` dip for paint
  * `RB` prepare to paint (hover in front of canvas)
  * `RT` paint (touch canvas with brush)

## Sending manual commands
Some commands don't have joystick mappings due to safety or laziness.  For example, to calibrate the motors, you need to send the string `c4`.  To do so, type the string into the "Low-level Serial Communication" text box (between "Connect" and checkboxes) and hit enter on keyboard or press the "Send" button.

--------------------------------------------------------------------------------

# Assembly

## Strut Channel Frame
1. Assemble frame / pulley mounting points
2. Place AIR box in frame
3. Place the wooden Arm base in a "home"-ish position in the frame.
4. Route the cables through the pulleys and clip them to the wooden Arm base.  Leave some slack in the cables - roughly 2 rotations-worth of each winch (6" or so).

## Electronics
1. Connect Teensy (Microcontroller aka MCU) in AIR box to laptop via USB
   1. At this point, 3 serial devices with similar names should appear on the computer.
2. Plug in power to AIR box
   * Before plugging in power, ensure that the AIR box power switch is `OFF`
   * Before plugging in power, make sure cables have some slack just in case
3. Inside AIR box, turn power switch to `1` -> `1 & 2` -> `2`
   * This should just involve turning the switch clockwise ("tighten a bolt") 3 clicks.  Give it a second or so between each click to allow the capacitors to pre-charge.
   > Checkpoint: you should hear/see the fan inside the AIR box power supply start spinning, and the 2 ODrives should have little green lights.
4. Plug in the Arm CAT5 cable
5. Plug in the Arm power cable
6. Un-press the E-STOP (if it is pressed)

--------------------------------------------------------------------------------

# Setup / Calibration

## Cable Robot
1. Bring up the HTML control panel and "connect" to the robot/Teensy.
2. Calibrate motors
   1. Ensure there is some slack in the cables.
   2. Ensure none of the motors are in an error state.
   3. In the control panel, type `c4` (and enter).
      1. You should hear a beep and all four motors should go to state 4, then 5, then 6, then 7, then 1.  Ensure that they all made it to state 7 before returning to 1.
3. Energize motors
   1. Manually wind the cables to make sure there isn't excessive slack.  A little slack is fine.
   2. Put the robot into `HOLD` mode by pressing `x` on the gamepad.
      1. The robot should pull the cables taught and maybe even lift up the wooden arm base an inch or so.
      2. BE READY TO ESTOP IF ANYTHING GOES UNEXPECTED HERE (the arm should not move more than an inch).
   3. Check that all the cables are routed through the pulleys correctly and not wedged between the pulley's wall.
4. Zero motors
   1. Manually move (with your hands) the wooden arm base to roughly the "home" position within the frame (resting on the AIR box or ground).
   2. Type `c14`.  On the top-left diagram, the black estimated position should be updated to the home position.
5. Calibrate robot
   1. In VSCode, open up `src/gerry09/gerry00_calibrate.ipynb`.
   2. Run through the notebook.  If you are not intimately familiar with the notebook, be sure to read the markdown instructions in the notebook.
   3. Enter settings
      
      If you didn't already run the very last cell of the calibration script, manually enter these settings for EVERY CONTROLLER YOU PLAN TO USE (using gamepad DPAD buttons or with `gs#` where # denotes 0/1/2 which controller you're using).  So, for example, send `gs0` then run all these.  Then send `gs1` and run all these again.
      * Max tension: `kKM300` (300N)
      * Distance threshold to stop tracking: `kd0.5` (0.5m)
      * Limits: `xLl1.0;xLd0.75;xLr2.0;xLu1.3;` (left, down, right, up)
      * Speed: `xs0.2;xS0.2` this is pretty fast but reasonable
6. Check
   
   Ensure everything is running as expected by switching to position control mode (joystick `Y`) and moving around a bit to test the robot is moving as you expect.

**CHECK TO MAKE SURE THE CABLES ARE ROUTED THROUGH THE PULLEYS CORRECTLY**

## Arm
In the RAW-AIR repo, (in a terminal e.g. in the built-in VSCode terminal) run `cd RAW-AIR/arm && python arm_server.py`.  This will start a server which listens for commands from the HTML control panel.  The arm server will then forward those commands to the Teensy which will forward them to the arm.  Mainly this python server exists because I was too lazy to implement the dynamixel protocol and IK in c++ or javascript and instead I implemented them in Python, so we need this server to get the different languages to talk to eachother.

You might need to refresh the HTML control panel.  The arm server terminal should print something like "rpc connected!!!!" when the HTML control panel connects to it.

If it's not connecting, double check that the `cable-robot/site/index.html` hasn't commented-out the `arm.js` import near the top of the file.

Test something simple like gamepad `LB` to make sure there's no errors and the arm is moving.

## iPad
In the `art_skills` repo, run `cd whiteboard && python server.py`.  This will start up a server that listens for the iPad to connect to it and forwards it to the cable robot control panel index.html.  Also, it will print out an ip address.  For example,
```
STARTING UP!!!
Serving whiteboard at: 143.215.94.191:5900
Serving robot at: localhost:5904
Serving robot at: localhost:5906
Serving fit server at ('::1', 5902, 0, 0), ('127.0.0.1', 5902)
```
In this case, we would copy `143.215.94.191`.  Then, edit the `art_skills/whiteboard/client/main.js` file and at the very top, paste the ip address into the `HOST` variable.

On the HTML control panel (after refresh), the debug console should show `Connected to ipad!!!` and the python should print `Robot connection opened!`.

If it's not connecting, double check that the `cable-robot/site/index.html` hasn't commented-out the `ipad.js` import near the top of the file.

~~

Next, launch a server to host the whiteboard/client/index.html (e.g. in VSCode, install the Live Server extension and click "Go Live" in the bottom right corner).  Then, open up the whiteboard on the iPad by going to `143.215.94.191:5500/whiteboard/client` (or whatever the IP address is).  Notice the Live Share extension defaults to port number 5500 (the "Go Live" icon says the port number), but if you're using a different port/server, use the correct port number.  Also, make sure that you do **not** do `https://143...`.  I don't know why but this doesn't work - just type it into the address bar with**out** the https.

On the python server, you should see `Whiteboard connection opened!`.

Test it by drawing a small shape in the center of the iPad.  You should see it appear in green on the HTML control panel and (assuming you're in iPad mode, Position control mode) the robot should start painting it.

--------------------------------------------------------------------------------

# Operation

This should be pretty straightforward if you walked through the setup.  Some random things you might want to do:

* ESTOP - press the red ESTOP button or click the right joystick.
* Clear drawings - especially when there's a big backlog of old trajectories that will take forever for the cable robot to finish, try refreshing the HTML control panel and iPad window.
* Reboot the Arm - sometimes the arm servos will overload or overheat or something.  I couldn't figure out a solution better than just unplugging and re-plugging the arm's power cable.
* Switch to joystick/iPad control - this can sometimes be confusing because, once an iPad (well actually the art_skills server.py) connects to the HTML control panel, it auto-switches to iPad control mode and the joystick no longer does anything.  Use DPAD LEFT+DOWN and UP+RIGHT to switch to joystick or iPad control modes respectively (see [Joystick Control](#joystick-control))
* Relax the robot arm - press the "Go Storage" button
* Manual control of the robot arm - see the bottom of `arm.js` and call any of those functions from the HTML debug console (or add a button to the HTML control panel) using `Arm.function_name`.
