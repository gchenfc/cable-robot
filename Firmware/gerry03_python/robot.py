import time
from enum import Enum
import struct
from can_simple import *
from enums import *
from typing import Callable, Optional, Iterable
from myTimer import MyTimer
from functools import partial
import datetime, json
import math
import numpy as np
import traceback

class Robot:
    def __init__(self, sendMsgFcn: Callable[[int, int, Optional[Iterable]], None]=None):
        self.sendMsgFcn = sendMsgFcn
        self.axes = [Axis(self, i) for i in range(4)]
        self.last_hb_t = [time.time() + 3,] * 4
        self.state = RobotState.IDLE
        self.queryTimers = [MyTimer(0.25, partial(self.query,cmd)) for cmd in
            [MSG_GET_MOTOR_ERROR,
             MSG_GET_ENCODER_ERROR,
             MSG_GET_SENSORLESS_ERROR,
             MSG_GET_ENCODER_ESTIMATES,
             MSG_GET_ENCODER_COUNT,
             MSG_GET_IQ,
             MSG_GET_SENSORLESS_ESTIMATES,
             MSG_GET_VBUS_VOLTAGE]]
        self.height, self.width, self.diag = None, None, None
        self.movingTimer = None
        
    def update(self):
        for timer in self.queryTimers:
            timer.update()
        if self.state == RobotState.MOVING:
            try:
                self.movingTimer.update()
            except:
                traceback.print_exc()
                print('error with moving timer')
        for axis in self.axes:
            axis.update()

    # control
    def dist(X, Y):
        dX = [x-y for x, y in zip(X, Y)]
        return math.sqrt(sum(dx*dx for dx in dX))
    def IK(self, pos, locs=None, pretension=0):
        if locs is None:
            locs = self.get_anchor_locs()
        return tuple(Robot.dist(loc, pos)-pretension for loc in locs)
    def FK(self, lengths=None, locs=None):
        if lengths is None:
            lengths = self.get_lengths()
        if locs is None:
            locs = self.get_anchor_locs()
        a = locs[0][0]-locs[3][0]
        b = abs(lengths[3])
        c = abs(lengths[0])
        try:
            gamma = math.acos((a*a + b*b - c*c) / (2*a*b))
        except:
            gamma = math.pi/4
        pos = (b*math.cos(gamma), b*math.sin(gamma))
        err = tuple(l - lhat for l,lhat in zip(lengths, self.IK(pos)))
        return pos, err
    def gotopos(self, pos, dry_run=False):
        if not self.is_calibrated():
            print('Please calibrate first')
            return
        if pos[0] < 0 or pos[0] > self.width or pos[1] < 0 or pos[1] > self.height:
            print('setpoint out of bounds')
            return
        ls = self.IK(pos, pretension=-1)
        print('{:.2f} * {:.2f} * {:.2f} * {:.2f}'.format(*ls))
        if dry_run:
            return
        self.setpoint_lengths = ls
        self.setpoint_pos = pos
        self.trajectory = []
        self.trajectoryi = 0
        tmppos, err = self.FK()
        while Robot.dist(tmppos, self.setpoint_pos) > 3:
            dx = [f-i for i,f in zip(tmppos, self.setpoint_pos)]
            magdx = math.sqrt(sum(d*d for d in dx))
            dx = [d / magdx for d in dx]
            tmppos = [p + 2 * d for p, d in zip(tmppos, dx)]
            # IK
            ls = self.IK(tmppos, pretension=-1)
            self.trajectory.append(ls)
        ls = self.IK(self.setpoint_pos, pretension=0.1)
        self.trajectory.append(ls)
        print('trajectory: {}'.format(self.trajectory))

        ctrl_modes = [3,3,1,1]
        for ai, axis in enumerate(self.axes):
            if axis.control_mode_believed != ctrl_modes[ai]:
                self.set_ctrl_mode(ai, ctrl_modes[ai])
                self.set_ctrl_mode(ai, ctrl_modes[ai])
                self.set_ctrl_mode(ai, ctrl_modes[ai])
            # self.set_setpoint(ai, ls[ai])
            # self.set_setpoint(ai, ls[ai])
            # self.set_setpoint(ai, ls[ai])
        self.state = RobotState.MOVING
        self.movingTimer = MyTimer(0.2, self.update_traj)
    def start_traj_x(self, traj):
        self.setpoint_pos = traj[-1]
        self.trajectory = []
        self.trajectoryi = 0
        for tmppos in traj:
            ls = self.IK(tmppos, pretension=-1)
            self.trajectory.append(ls)
        ctrl_modes = [3,3,1,1]
        for ai, axis in enumerate(self.axes):
            if axis.control_mode_believed != ctrl_modes[ai]:
                self.set_ctrl_mode(ai, ctrl_modes[ai])
                self.set_ctrl_mode(ai, ctrl_modes[ai])
                self.set_ctrl_mode(ai, ctrl_modes[ai])
        self.state = RobotState.MOVING
        self.movingTimer = MyTimer(0.2, self.update_traj)
    def update_traj(self):
        if True or self.trajectoryi >= len(self.trajectory):
            ls = self.trajectory[-1]
            print('attempting to reach final state {}'.format(self.setpoint_pos))
        else:
            ls = self.trajectory[self.trajectoryi]
            if (max(abs(l - ax.get_pos()) for l,ax in zip(ls, self.axes)) < 3):
                self.trajectoryi += 1
            print('checkpoint {} of {}'.format(self.trajectoryi, len(self.trajectory)))

        curpos, err = self.FK()
        d = Robot.dist(curpos, self.setpoint_pos)
        # ls = (0.2, 0.2, ls[2], ls[3])
        ls = (ls[0], ls[1], 0.4, 0.4)
        if d < 3 and (self.trajectoryi >= (len(self.trajectory)-3)):
            self.state = RobotState.IDLE
            for ai in range(4):
                self.set_setpoint(ai, ls[ai])
            print('arrived!!!')
        else:
            for ai in range(4):
                self.set_setpoint(ai, ls[ai])

    # calibration
    def is_calibrated(self):
        return not (self.height is None or
                    self.width is None or
                    self.diag is None or
                    any(axis.location is None for axis in self.axes))
    def set_calib_length(self, axis, other_axis):
        self.axes[axis].calib_lengths[other_axis] = self.axes[axis].pos
        return True
    def home_axis(self, axis):
        self.axes[axis].home()
    def print_calibration(self):
        lengths, dims, _ = self.export_calibration(export=False)
        for i, ls in enumerate(lengths):
            print('\taxis {} lengths: {}'.format(i, str(ls)))
        print('frame height: {:.2f}\twidth: {:.2f}\tdiagonal: {:.2f}'.format(*dims))
    def export_calibration(self, export=True):
        h,w,d = self.calculate_calibration(apply=False)
        data = {
            'calib_lengths':
                {i: axis.calib_lengths for i, axis in enumerate(self.axes)},
            'geometry':
                {'height': h, 'width': w, 'diag': d}
            }
        fname = None
        if export:
            datajson = json.dumps(data)
            fname = 'calib/' + datetime.datetime.now().strftime('%Y%m%d_%H%M') + '.json'
            with open(fname, 'w') as f:
                f.write(datajson)
            with open('calib/latest.json', 'w') as f:
                f.write(datajson)
        return tuple(axis.calib_lengths for axis in self.axes), (h,w,d), fname
    def import_calibration(self, fname='calib/latest.json'):
        with open(fname, 'r') as f:
            data = json.load(f)
        for i, axis in enumerate(self.axes):
            axis.calib_lengths = data['calib_lengths']['{}'.format(i)]
        return data
    def calculate_calibration(self, apply=True):
        #         0, H, W, Diag
        order = [[0, 1, 3, 2],
                 [1, 0, 2, 3],
                 [2, 3, 1, 0],
                 [3, 2, 0, 1]]
        lengths = [axis.calib_lengths for axis in self.axes]
        for ai, axis in enumerate(self.axes):
            z = lengths[ai][order[ai][0]]
            lengths[ai] = [l - z for l in lengths[ai]]
            if apply:
                axis.zero = z
        height = abs(sum(lengths[ai][order[ai][1]] for ai in range(4))) / 4
        width = abs(sum(lengths[ai][order[ai][2]] for ai in range(4))) / 4
        diag = abs(sum(lengths[ai][order[ai][3]] for ai in range(4))) / 4
        if ((width*width + height*height)**0.5 - diag) > (0.01*diag):
            print('WARNING!!! calibration out by more than 1%')
        if apply:
            self.height, self.width, self.diag = height, width, diag
            self.axes[0].location = (width, 0)
            self.axes[1].location = (width, height)
            self.axes[2].location = (0, height)
            self.axes[3].location = (0, 0)
        return height, width, diag
    
    # convenience functions
    def get_anchor_locs(self):
        return [axis.location for axis in self.axes]
    def get_lengths(self):
        return [axis.get_pos() for axis in self.axes]
    def get_pos(self):
        if not self.is_calibrated():
            return (float('nan'),)*2, (float('nan'),)*4
        return self.FK()
    def get_cmd_lengths(self):
        return [axis.commanded_pos for axis in self.axes]

    # send message
    def query(self, cmd):
        for axis in self.axes:
            axis.query(cmd)
    def estop(self):
        self.query(MSG_ODRIVE_ESTOP)
        self.query(MSG_ODRIVE_ESTOP)
        self.query(MSG_ODRIVE_ESTOP)
    def clear_errors(self):
        self.query(MSG_CLEAR_ERRORS)
    def reboot(self, device: int):
        self.sendMsgFcn(device // 2, MSG_RESET_ODRIVE)
    def set_state(self, axis, state):
        self.axes[axis].set_state(state)
    def set_ctrl_mode(self, axis, mode):
        self.axes[axis].set_ctrl_mode(mode)
    def set_setpoint(self, axis, setpoint):
        self.axes[axis].set_setpoint(setpoint)

    # receive message
    def callback(self, node, cmd, data: bytes):
        if (node > 3):
            print('\tinvalid node! - {} {} {}'.format(node, cmd, data.hex()))
            return
        self.axes[node].readCAN(cmd, data)

class RobotState(Enum):
    IDLE = 1
    ZEROING = 2
    CALIBRATING = 3
    MOVING = 4

class Axis:
    def __init__(self, parent, node):
        self.parent = parent
        self.sendMsgFcn = parent.sendMsgFcn
        self.node = node

        self.last_hb_t = time.time() + 3
        self.errorcode = float('nan')
        self.state = 0
        self.pos = float('nan')
        self.vel = float('nan')
        self.current_setpoint = float('nan')
        self.current_measured = float('nan')
        self.motorError = float('nan')
        self.encoderError = float('nan')
        self.sensorlessError = float('nan')
        self.encoderCount = (float('nan'), float('nan')) # shadow, count
        self.sensorlessEst = (float('nan'), float('nan')) # pos, vel
        self.voltage = float('nan')

        self.commanded_state = 0
        self.reached_state = True
        self.control_mode_believed = 1
        self.calib_lengths = [float('nan'),]*4
        self.zero = float('nan')
        self.location = None
        self.commanded_pos = float('nan')

        self.resendCtrlModeTimer = MyTimer(0.5, self.resend_ctrl_mode)
    
    def update(self):
        if not self.reached_state:
            if self.state != self.commanded_state and \
               self.commanded_state != 3: # calibration
                self.set_state(self.commanded_state)
            else:
                self.reached_state = True
        self.resendCtrlModeTimer.update()

    # util
    def get_pos(self):
        return self.pos_raw2robot(self.pos)
    def home(self, offset=2.54/(2.54*math.pi)):
        self.zero = self.pos + offset
        ls = self.calib_lengths
        self.calib_lengths = [l - ls[self.node] + self.zero for l in ls]

    # send message
    def query(self, cmd):
        self.sendMsgFcn(self.node, cmd)
    def set_state(self, state):
        self.sendMsgFcn(self.node, MSG_SET_AXIS_REQUESTED_STATE, state)
        self.commanded_state = state
        self.reached_state = False
    def set_ctrl_mode(self, mode):
        self.sendMsgFcn(self.node, MSG_SET_CONTROLLER_MODES, [mode, 1]) # always passthrough
        self.control_mode_believed = mode
    def resend_ctrl_mode(self):
        self.set_ctrl_mode(self.control_mode_believed)
    def set_setpoint(self, setpoint):
        mode = self.control_mode_believed
        self.commanded_pos = float('nan')
        if mode == 1:
            self.sendMsgFcn(self.node, MSG_SET_INPUT_TORQUE, [setpoint])
        elif mode == 2:
            self.sendMsgFcn(self.node, MSG_SET_INPUT_VEL, [setpoint, 0])
        elif mode == 3:
            self.sendMsgFcn(self.node, MSG_SET_INPUT_POS, [self.pos_robot2raw(setpoint), 0, 0])
            self.commanded_pos = setpoint
        else:
            print('unknown mode during setpoint command: {}'.format(mode))
    
    # receive message
    def readCAN(self, cmd, data):
        # unpack everything at first
        f1, f2 = struct.unpack('<2f', data)  # little endian, 2 floats
        i1, i2 = struct.unpack('<2i', data)  # little endian, 2 ints (4 bytes)
        p, v, t = struct.unpack('<fhh', data)  # little endian, float-short-short
        # match command
        if (cmd == MSG_ODRIVE_HEARTBEAT):
            if i2 <= AxisStateEnum.AXIS_STATE_HOMING.value and i2 >= 0:
                self.errorcode = i1
                self.state = i2
                self.last_hb_t = time.time()
        elif (cmd in [MSG_ODRIVE_ESTOP,
                    MSG_START_ANTICOGGING,
                    MSG_RESET_ODRIVE,
                    MSG_CLEAR_ERRORS]):
            print('\tThis messastatege should not be coming from the slave')
        # query info
        elif (cmd == MSG_GET_MOTOR_ERROR):
            self.motorError = i1
        elif (cmd == MSG_GET_ENCODER_ERROR):
            self.encoderError = i1
        elif (cmd == MSG_GET_SENSORLESS_ERROR):
            self.sensorlessError = i1
        elif (cmd == MSG_GET_ENCODER_ESTIMATES):
            self.pos = f1
            self.vel = f2
        elif (cmd == MSG_GET_ENCODER_COUNT):
            self.encoderCount = (i1, i2)
        elif (cmd == MSG_GET_IQ):
            self.current_setpoint = f1
            self.current_measured = f2
        elif (cmd == MSG_GET_SENSORLESS_ESTIMATES):
            self.sensorlessEst = (f1, f2)
        elif (cmd == MSG_GET_VBUS_VOLTAGE):
            self.voltage = f1
        elif(cmd in [MSG_SET_AXIS_NODE_ID,
                    MSG_SET_AXIS_REQUESTED_STATE,
                    MSG_SET_AXIS_STARTUP_CONFIG,
                    MSG_SET_CONTROLLER_MODES,
                    MSG_SET_INPUT_POS,
                    MSG_SET_INPUT_VEL,
                    MSG_SET_INPUT_TORQUE,
                    MSG_SET_VEL_LIMIT,
                    MSG_SET_TRAJ_VEL_LIMIT,
                    MSG_SET_TRAJ_ACCEL_LIMITS,
                    MSG_SET_TRAJ_INERTIA]):
            print('\tThis message should not be coming from the slave')
        else:
            print('\tCommand not recognized!')
    
    # internal utils
    def pos_robot2raw(self, pos):
        if not math.isnan(self.zero):
            return -pos + self.zero
        return -pos
    def pos_raw2robot(self, pos):
        if not math.isnan(self.zero):
            return -(pos - self.zero)
        return -pos
