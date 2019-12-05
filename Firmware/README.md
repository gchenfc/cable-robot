# Cable Robot Firmware

Firmware exists for both the ODrive and Teensy.  Assuming all libraries are already installed and ODrive firmware is up to date, flash `Firmware/gerry00_roscan` onto the Teensy to control the system using ROS.

## ODrive
TODO(James): document this stuff  
ODrive firmware uses [Wetmelon's fork](https://github.com/Wetmelon/ODrive/tree/feature/CAN) since mainline hasn't merged CAN yet.  In the future, expect that we will need some additional firmware modifications so we may make our own fork.  
Flashing is easy using
```
cd ./Firmware/ODrive_firmware/Firmware
make
odrivetool dfu ./build/ODriveFirmware.hex
```
See [odriverobotics](https://docs.odriverobotics.com/developer-guide.html) for additional info.

## Teensy
The following libraries are required for the Teensy's code:
* **Teensyduino** - not really a library, but you have to install [Teensyduino](https://www.pjrc.com/teensy/td_download.html) to flash code to the Teensy.  It's an Arduino add-on (so you need to install Arduino as well)
* **rosserial** - follow the instructions [here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) to install rosserial.  Pay extra attention to section 2.2.  The result should be a folder called "ros_lib" in your Arduino/libraries folder.  A sample ros_lib folder is also provided here in this repo in `Firmware/libraries`.
* **FlexCAN** - modified from [collin80's fork](https://github.com/collin80/FlexCAN_Library).
  There are so many FlexCAN forks for Teensy that we need to link the right one in this repo.  The
  PR has been stale for more than 2 years now, so we'll just keep the library source code in this repo at `Firmware/libraries/FlexCAN`.  Installation is a bit tricky: you have to copy the `Firmware/libraries/FlexCAN` folder into the hardware folder inside the Teensyduino installation.  So, for example, `/Applications/Arduino.app/Contents/Resources/Java/hardware/teensy/avr/libraries`.  This is especially important if there is an existing FlexCAN folder in there because Arduino will default to using the FlexCAN in there instead of our own FlexCAN.
* **ChibiOS-RT** - The [ChibiOS](chibios.org) library for Teensy [(github)](https://github.com/greiman/ChRt) needs to be installed.  Easiest way is to copy the `Firmware/libraries/ChRt` folder into `{path to Arduino}/libraries/`.  For example, `cp -r Firmware/libraries/ChRt ~/Arduino/libraries/`
* **wifi** - TODO(James): wifi stuff