import dataclasses
import serial
import time
from enum import Enum
import numpy as np
import parse
import copy


@dataclasses.dataclass
class ControllerState:
    time_us: int
    state: int
    cur_x: float
    cur_y: float
    cur_th: float
    set_x: float
    set_y: float
    set_th: float

    def __repr__(self):
        return f'({self.time_us / 1e6} {self.state} ({self.cur_x}, {self.cur_y}) ({self.set_x}, {self.set_y}))'


@dataclasses.dataclass
class MotorState:
    error: int
    state: int
    length: float
    lengthdot: float

    def __repr__(self):
        return f'({self.error} {self.state} {self.length} {self.lengthdot})'


class CableRobot:

    class State(Enum):
        UNKNOWN = 0
        HOLD = 1
        RELEASE = 2
        TRACKING = 3

    def __init__(self,
                 port='/dev/tty.usbmodem100994303',
                 baud=1000000,
                 timeout=0,
                 write_timeout=0.001,
                 silent=True,
                 print_raw=False,
                 initial_msg: str = None,
                 **kwargs):
        self.ser = serial.Serial(port, baud, timeout=timeout, write_timeout=write_timeout)
        self.ser.reset_input_buffer()
        self.silent = silent

        if print_raw:
            self.send('d1')
        else:
            self.send('d0')
        if initial_msg is not None:
            self.send(initial_msg)

        self.t_prev = time.time()
        self.state = CableRobot.State.UNKNOWN
        self.meas_state = CableRobot.State.UNKNOWN
        self.cur_xy = None
        self.setpoint_xy = np.array([1.0, 0.9])
        self.vel_xy = np.array([0, 0])
        self.ser_buf = ''
        self.SERIAL_FMT = {
            'controller':
                parse.compile(
                    '{time_us:d} - {state:d}: {cur_th:f} {cur_x:f} {cur_y:f} - {set_th:f} {set_x:f} {set_y:f}'
                ),
            'motor':
                parse.compile('{error:d} {state:d} {length:f} {lengthdot:f}'),
            'spray':
                parse.compile('{spray:d}')
        }
        self.log_data = {'controller': None, 'motors': [None, None, None, None], 'spray': None}
        self.all_data = []

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.ser.close()
        return False

    def update(self):
        t = time.time()
        dt = t - self.t_prev
        self.t_prev = t

        s = self.read()
        if not self.silent and s:
            print(s, end='')

        if self.state == CableRobot.State.TRACKING:
            self.send_tracking_position(*(self.vel_xy * dt), relative=True)
            # self.update_tracking_position(dt)

    def send(self, str):
        self.ser.write(bytes(str + '\n', 'utf-8'))
        if not self.silent:
            print("(cablerobot) sent: {:}\\n".format(str))

    def clear_errors(self):
        self.send('g0')
        self.state = CableRobot.State.UNKNOWN

    def estop(self):
        self.send('0n2c;3n2c;1n2c;2n2c')
        self.state = CableRobot.State.UNKNOWN

    def set_mode_hold(self):
        self.send('g6;g8')
        self.state = CableRobot.State.HOLD

    def set_mode_release(self):
        self.send('g7')
        self.state = CableRobot.State.RELEASE

    def set_mode_tracking(self, update_setpoint=True):
        self.send('g1;g2')
        if update_setpoint and (self.cur_xy is not None):
            self.setpoint_xy = self.cur_xy
        self.state = CableRobot.State.TRACKING

    def reached_tracking_position(self, tol=0.03):
        dx = self.setpoint_xy - self.cur_xy
        return np.sum(np.square(dx)) < np.square(tol)

    def update_tracking_position(self):

        def towards(cur, target, dist):
            dx = target - cur
            if np.linalg.norm(dx) >= dist:
                return cur + (dx / np.linalg.norm(dx) * dist)
            else:
                return target

        if self.state is not CableRobot.State.TRACKING:
            print('(cablerobot) Error: cannot send tracking commands when not in tracking mode')
            return
        # self.cur_xy = towards(self.cur_xy, self.setpoint_xy, dt)
        self.send('ta{:f},{:f}'.format(*self.setpoint_xy))

    def send_tracking_position(self, x, y, relative=False):
        if self.state is not CableRobot.State.TRACKING:
            print('(cablerobot) Error: cannot send tracking commands when not in tracking mode')
            return
        if relative:
            self.setpoint_xy += np.array([x, y])
            # self.send('tr{:f};td{:f}'.format(x, y))
        else:
            self.setpoint_xy = np.array([x, y])
            # self.send('ta{:f},{:f}'.format(x, y))
        self.update_tracking_position()

    def read(self):
        toret = ''
        while self.ser.in_waiting > 0:
            toret += self.ser.read_all().decode('utf-8')
        self.ser_buf += toret
        self.parse_buf()
        return toret

    def parse_buf(self):
        lines = self.ser_buf.split('\n')
        for line in lines[:-1]:
            cdata, mdata, _, controller_state, motor_states = parse_line(line, self.SERIAL_FMT)
            if cdata is None or mdata is None or controller_state is None or motor_states is None:
                continue
            self.cur_xy = np.array([cdata['cur_x'], cdata['cur_y']])
            self.log_data['controller'] = controller_state
            self.log_data['motors'] = motor_states
            self.all_data.append(copy.deepcopy(self.log_data))

            # self.setpoint_xy = np.array([cdata['set_x'], cdata['set_y']])
            # if cdata['state'] == 1:
            #     self.meas_state = CableRobot.State.RELEASE
            # elif cdata['state'] == 6:
            #     self.meas_state = CableRobot.State.HOLD
            # elif cdata['state'] == 3:
            #     self.meas_state = CableRobot.State.TRACKING
            # else:
            #     self.meas_state = CableRobot.State.UNKNOWN

            # print('(cablerobot) parsed data: {:}\t{:}\t{:}'.format(self.meas_state, self.cur_xy,
            #                                                        self.setpoint_xy))

        self.ser_buf = lines[-1]


SERIAL_FMT = {
    'controller':
        parse.compile(
            '{time_us:d} - {state:d}: {cur_th:f} {cur_x:f} {cur_y:f} - {set_th:f} {set_x:f} {set_y:f}'
        ),
    'motor':
        parse.compile('{error:d} {state:d} {length:f} {lengthdot:f}'),
    'spray':
        parse.compile('{spray:d}')
}


def parse_line(line, serial_fmt=SERIAL_FMT):
    line = line.strip()
    chunks = line.split('|')
    if len(chunks) != 6:
        print('not correct number of chunks!', chunks)
        return None, None, None, None, None
    cdata = serial_fmt['controller'].parse(chunks[0].strip())
    mdata = [serial_fmt['motor'].parse(chunk.strip()) for chunk in chunks[1:-1]]
    sdata = serial_fmt['spray'].parse(chunks[-1].strip())
    try:
        controller_state = ControllerState(**cdata.named)
        motor_states = [MotorState(**mdatum.named) for mdatum in mdata]
    except AttributeError as e:
        print('parse error')
        controller_state = None
        motor_states = None

    return cdata, mdata, sdata, controller_state, motor_states


if __name__ == '__main__':
    print('Collecting data for 5 seconds...')

    robot = CableRobot()
    tstart = time.time()
    while True:
        robot.update()
        time.sleep(0.01)
        if (time.time() - tstart > 5):
            break

    print('Done collecting data.  Plotting...')

    motor_ls = np.array([[m.length for m in datum['motors']] for datum in robot.all_data])
    motor_ldots = np.array([[m.lengthdot for m in datum['motors']] for datum in robot.all_data])

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(6, 9))
    axes[0].plot(motor_ls)
    axes[1].plot(motor_ldots)
    plt.show()
