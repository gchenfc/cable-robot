import pyglet
import time

from communicate import CableRobot, MotorState, ControllerState
import numpy as np
import matplotlib.pyplot as plt
import time

def get_tablet():
    tablet = None
    for device in pyglet.input.get_devices():
        if device.name is None or 'Tablet' not in device.name:
            print(device.name)
            continue
        print(device.name, type(device), device.get_controls())
        if len(device.get_controls()) > 5:
            tablet = device
            break
    return tablet

def get_controls(tablet):
    controls = tablet.get_controls()
    return controls

def callback(controls):
    c_x, c_y = controls[-3], controls[-2]
    x = (c_x.value - c_x.min) / (c_x.max - c_x.min)
    y = 1 - (c_y.value - c_y.min) / (c_y.max - c_y.min)
    print(x, y)
    return x, y
    # for control in controls:
    #     if isinstance(control, pyglet.input.base.Button):
    #         print(control.value)
    #     else:
    #         print(control.value) # control.min, control.max

def update_cablerobot(robot, x, y):
    robot.send('ta{:},{:}'.format(1.5 + (x-0.5), 1.2 + (y-0.5)))

def main():
    tablet = get_tablet()
    controls = get_controls(tablet)
    robot = CableRobot(print_raw=False, write_timeout=None, initial_msg='d10,100')

    # Set up gui
    window = pyglet.window.Window(1020, 576)

    try:
        canvas = tablet.open(window)
    except pyglet.input.DeviceException:
        print('Failed to open tablet on window')

    print('Opened window')

    @controls[-2].event
    def on_change(_):
        x, y = callback(controls)
        update_cablerobot(robot, x, y)

    pyglet.app.run()

    robot.ser.close()

if __name__ == '__main__':
    main()
