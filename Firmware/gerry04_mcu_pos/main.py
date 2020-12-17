from robot import Robot
from mySerial import MySerial
from gui import MyGUI

quit = False
def exit_cb():
    global quit
    quit = True
def main():
    with MySerial() as ser:
        robot = Robot(ser.writeCan)
        gui = MyGUI(robot, exit_cb, ser.writeRaw)
        gui.start()
        while not quit:
            robot.update()
            ser.readCan(robot.callback)
            ser.update()

if __name__ == "__main__":
    main()