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

## Operation

### Things to periodically check
* Main battery balance
* Carriage battery/BEC temperature
* Carriage battery voltage
