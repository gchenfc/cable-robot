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
from typing import Callable, Tuple

class Robot:
    def __init__(self, sendMsgFcn: Callable[[int, int, Optional[Iterable]], None]=None):
        # wrapper for sendMsgFcn
        self.is_held = False
        self.msgOutBuffer = []
        self.sendMsgFcn_raw = sendMsgFcn

        self.axes = [Axis(self, i) for i in range(4)]
        self.last_hb_t = [time.time() + 3,] * 4
        self.state = RobotState.IDLE
        self.queryTimers = [MyTimer(0.01, partial(self.query,cmd)) for cmd in
            [MSG_GET_ENCODER_ESTIMATES,
             MSG_GET_IQ]]
        self.queryTimers.append(*[MyTimer(1, partial(self.query,cmd)) for cmd in
            [
            #  MSG_GET_MOTOR_ERROR,
            #  MSG_GET_ENCODER_ERROR,
            #  MSG_GET_SENSORLESS_ERROR,
            #  MSG_GET_ENCODER_COUNT,
            #  MSG_GET_SENSORLESS_ESTIMATES,
             MSG_GET_VBUS_VOLTAGE]])
        self.height, self.width, self.diag = None, None, None
        self.movingTimer = None

        # ack on held commands
        # TODO(gerry): held commands with only 1 msg don't get acked and
        # causes infinite resending waiting for ack
        self.msgBufferAcked = True
        self.msgOutBufferLast = []
        # self.msgBufferAckTimer = MyTimer(0.05, self.resendLastHold)

        # recording
        self.is_recording = False
        self.recording_timer = None
        self.recording_data = []

        self.status = ''
        
        self.paintIsOn = False
        self.paintTimer = MyTimer(1.25, self.updatePaint)
    def update(self):
        for timer in self.queryTimers:
            timer.update()
        # if not self.msgBufferAcked:
        #     self.msgBufferAckTimer.update()
        self.paintTimer.update()
        if self.state == RobotState.MOVING:
            try:
                self.movingTimer.update()
            except:
                traceback.print_exc()
                print('error with moving timer')
        for axis in self.axes:
            axis.update()
        if self.is_recording:
            self.recording_timer.update()

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
        self.trajectory.append(self.setpoint_pos)
        print('trajectory: {}'.format(self.trajectory))

        self.dir = self.setpoint_pos[0] > self.FK()[0][0]
        # if self.dir:
        #     ctrl_modes = [3,3,1,1]
        #     vel_lims = [10,10,20,20]
        # else:
        #     ctrl_modes = [1,1,3,3]
        #     vel_lims = [20,20,10,10]
        ctrl_modes = [1,1,1,1]
        vel_lims = [2, 2, 2, 2]
        self.sendMsgs([*[(self.set_ctrl_mode, (ai, ctrl_mode)) for ai, ctrl_mode in enumerate(ctrl_modes)],
                       *[(self.set_vel_limit, (ai, vel_lim)) for ai, vel_lim in enumerate(vel_lims)]])
        self.state = RobotState.MOVING
        self.xerr_prev = None
        self.tprev = None
        self.xerr_int = [0, 0]
        self.movingTimer = MyTimer(0.01, self.update_traj)
    def start_traj_x(self, traj):
        self.setpoint_pos = traj[-1]
        self.trajectory = []
        self.trajectoryi = 0
        for tmppos in traj:
            self.trajectory.append(tmppos)
        self.dir = self.trajectory[0][0] > self.FK()[0][0]
        if self.dir:
            ctrl_modes = [3,3,1,1]
            vel_lims = [10,10,20,20]
        else:
            ctrl_modes = [1,1,3,3]
            vel_lims = [20,20,10,10]
        self.sendMsgs([*[(self.set_ctrl_mode, (ai, ctrl_mode)) for ai, ctrl_mode in enumerate(ctrl_modes)],
                       *[(self.set_vel_limit, (ai, vel_lim)) for ai, vel_lim in enumerate(vel_lims)]])
        self.state = RobotState.MOVING
        self.movingTimer = MyTimer(0.05, self.update_traj)
    def update_traj(self):
        # self.trajectoryi = len(self.trajectory)
        if self.trajectoryi >= len(self.trajectory):
            pos = self.trajectory[-1]
            ls = self.IK(pos, pretension=0.1)
            # print('attempting to reach final state {}'.format(self.setpoint_pos))
        else:
            pos = self.trajectory[self.trajectoryi]
            ls = self.IK(pos, pretension=0.1)
            if (max(abs(l - ax.get_pos()) for l,ax in zip(ls, self.axes)) < 3):
                self.trajectoryi += 1
            # print('checkpoint {} of {}'.format(self.trajectoryi, len(self.trajectory)))

        curpos, err = self.FK()
        d = Robot.dist(curpos, self.trajectory[-1])
        msgs = [] # TODO(gerry): this has trouble sending more than 8 messages at a time I think - direction changes are jerky
        newdir = pos[0] > curpos[0]
        # print('curdir: {}, newdir: {}'.format(self.dir, newdir))
        # controller
        xerr = [c-p for p,c in zip(pos, curpos)]
        tnow = time.time()
        if self.tprev is None:
            dt = 1e-3
        else:
            dt = tnow - self.tprev
        self.tprev = tnow
        if self.xerr_prev is None:
            self.xerr_prev = xerr
        xdot = [(x-xp) / dt for x, xp in zip(xerr, self.xerr_prev)]
        ilim = 10
        self.xerr_int = [min(max(i + x*dt, -ilim), ilim) for i, x in zip(self.xerr_int, xerr)]
        Kp, Ki, Kd = 0.05, 0.05, 0
        Kp, Ki, Kd = 0.05, 0, 0
        u = [-Kp * x - Ki * i - Kd * d for x, i, d in zip(xerr, self.xerr_int, xdot)]
        ulim = 0.7
        u = [min(max(ui, -ulim), ulim) for ui in u]
        ls = [0.2,]*4
        if u[0] > 0:
            ls[0] += u[0]
            ls[1] += u[0]
        else:
            ls[2] += abs(u[0])
            ls[3] += abs(u[0])
        if u[1] > 0:
            ls[1] += u[1]
            ls[2] += u[1]
        else:
            ls[0] += abs(u[1])
            ls[3] += abs(u[1])
        if d < 1 and (self.trajectoryi >= (len(self.trajectory)-2)):
            ls = [0.2,]*4
            self.state = RobotState.IDLE
            print('arrived!!!')
        else:
            pass
            self.status = 'not arrived yet: d={}'.format(d)
        for ai, l in enumerate(ls):
            msgs.append((self.set_setpoint, (ai, l)))
    def updatePaint(self):
        self.sendMsgFcn(9, self.paintIsOn, now=True)
    # recording
    def start_recording(self):
        self.is_recording = True
        self.recording_timer = MyTimer(0.1, self.record_sample)
    def record_sample(self):
        data = {ax.node: ax.export_sample() for ax in self.axes}
        data['t'] = time.time()
        data['x'], data['xerr'] = self.FK()
        self.recording_data.append(data)
    def stop_recording(self):
        data = {'data': self.recording_data,
                'config': {ax.node: ax.export_config() for ax in self.axes}}
        datajson = json.dumps(data)
        fname = 'recordings/' + datetime.datetime.now().strftime('%Y%m%d_%H%M') + '.json'
        with open(fname, 'w') as f:
            f.write(datajson)
        with open('recordings/latest.json', 'w') as f:
            f.write(datajson)
        print('wrote recordig to {:}'.format(fname))
        self.is_recording = False
        self.recording_data = []

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
        hypot = math.sqrt(dims[0]*dims[0] + dims[1]*dims[1])
        print('frame height: {:.2f}\twidth: {:.2f}\tdiagonal: {:.2f}\tpredicted diagonal: {:.2f}'.format(*dims, hypot))
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
        self.sendMsgFcn(device * 2, MSG_RESET_ODRIVE)
    def set_state(self, axis, state):
        self.axes[axis].set_state(state)
        print('setting state')
    def set_ctrl_mode(self, axis, mode):
        self.axes[axis].set_ctrl_mode(mode)
    def set_setpoint(self, axis, setpoint):
        self.axes[axis].set_setpoint(setpoint)
    def set_vel_limit(self, axis, limit):
        self.axes[axis].set_vel_limit(limit)
    def sendMsgFcn(self, node, cmd, data=None, now=False):
        if not self.is_held or now:
            self.sendMsgFcn_raw(node, cmd, data)
        else:
            self.msgOutBuffer.append((node, cmd, data))
    def sendMsgs(self, fcn_args: Iterable[Tuple[Callable, Iterable]]):
        # fcn_args is of the form
        # [(function1, (args1)), (function2, (args2)), ...]
        tmphold = self.is_held
        self.hold()
        for fcn, args in fcn_args:
            fcn(*args)
        self.sendhold()
        self.is_held = tmphold
    def hold(self):
        self.msgOutBuffer = []
        self.is_held = True
    def unhold(self):
        self.msgOutBuffer = []
        self.is_held = False
    def sendhold(self):
        if not self.is_held:
            print("nothing to unhold")
        else:
            nodes = [buf[0] for buf in self.msgOutBuffer]
            cmds = [buf[1] for buf in self.msgOutBuffer]
            datas = [buf[2] for buf in self.msgOutBuffer]
            self.sendMsgFcn_raw(nodes, cmds, datas, islist=True)
            # self.msgBufferAcked = False
            # self.msgOutBufferLast = self.msgOutBuffer.copy()
            # self.msgOutBuffer = []
    def resendLastHold(self):
        # nodes = [buf[0] for buf in self.msgOutBufferLast]
        # cmds = [buf[1] for buf in self.msgOutBufferLast]
        # datas = [buf[2] for buf in self.msgOutBufferLast]
        # self.sendMsgFcn_raw(nodes, cmds, datas, islist=True)
        pass

    # receive message
    def callback(self, node, cmd, data: bytes, ack=False):
        if ack:
            print("ACKED!!!!!")
            self.msgBufferAcked = True
            return
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
        pass
        # note: resendCtrlModeTimer and resend command are replaced by "hold" functionality

    # util
    def get_pos(self):
        return self.pos_raw2robot(self.pos)
    def home(self, offset=2.54/(2.54*math.pi)):
        self.zero = self.pos + offset
        ls = self.calib_lengths
        self.calib_lengths = [l - ls[self.node] + self.zero for l in ls]
    def export_sample(self):
        return {'last_hb_t': self.last_hb_t,
                'errorcode': self.errorcode,
                'state': self.state,
                'pos': self.get_pos(),
                'pos_raw': self.pos,
                'vel': self.vel,
                'Imeas': self.current_measured,
                'Iset': self.current_setpoint,
                'motorError': self.motorError,
                'encoderError': self.encoderError,
                'sensorlessError': self.sensorlessError,
                'encoderCount': self.encoderCount,
                'sensorlessEst': self.sensorlessEst,
                'voltage': self.voltage,
                'control_mode_believed': self.control_mode_believed}
    def export_config(self):
        return {'zero': self.zero,
                'location': self.location,
                'calib_lengths': self.calib_lengths}
    # send message
    def query(self, cmd):
        self.sendMsgFcn(self.node, cmd, now=True)
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
    def set_vel_limit(self, limit):
        self.sendMsgFcn(self.node, MSG_SET_VEL_LIMIT, [limit])
    
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
