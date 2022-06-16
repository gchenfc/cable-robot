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

def callback(controls, debug=False):
    """Returns (x, y, ispressed)"""
    # 0: Pen touching tablet (bool - ispressed)
    # 1: Pen button 1
    # 2: Pen button 2
    # 3-6: ???
    # 7: x
    # 8: y (top is 0)
    # 9: pen pressure
    c_x, c_y = controls[-3], controls[-2]
    x = (c_x.value - c_x.min) / (c_x.max - c_x.min)
    y = 1 - (c_y.value - c_y.min) / (c_y.max - c_y.min)
    if debug:
        print(x, y)
        for control in controls:
            if isinstance(control, pyglet.input.base.Button):
                print(control.value, end='\t')
            else:
                print(control.value, end='\t') # control.min, control.max
        print()
    return x, y, controls[0].value

def update_cablerobot(robot, x, y):
    robot.send('ta{:},{:}'.format(1.5 + (x-0.5), 1.2 + (y-0.5)))

def main():
    tablet = get_tablet()
    controls = get_controls(tablet)

    # Set up gui
    window = pyglet.window.Window(1020, 576, visible=True)

    try:
        canvas = tablet.open(window)
    except pyglet.input.DeviceException:
        print('Failed to open tablet on window')
        return
    else:
        print('Opened tablet on window')

    @window.event
    def on_key_press(symbol, modifiers):
        if symbol == ord('w') and (modifiers & pyglet.window.key.MOD_ACCEL):  # cmd-W
            window.close()

        if symbol == ord('q'):
            window.close()

    with CableRobot(print_raw=False, write_timeout=None, initial_msg='d10,100') as robot:
        @controls[-2].event
        @controls[-3].event
        @controls[-1].event
        def on_change(_):
            x, y, is_pressed = callback(controls)
            print(x, y)
            if is_pressed:
                update_cablerobot(robot, x, y)
                print(x, y)

        pyglet.app.run()

if __name__ == '__main__':
    main()
