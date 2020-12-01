import time
from enum import Enum
import struct
from can_simple import *
from enums import *
from typing import Callable, Optional, Iterable
from myTimer import MyTimer
from functools import partial
import datetime, json

class Robot:
    def __init__(self, sendMsgFcn: Callable[[int, int, Optional[Iterable]], None]=None):
        self.zeros = [0,0,0,0]
        self.mountingPts = None
        self.axes = [AxisState() for i in range(4)]
        self.last_hb_t = [time.time() + 3,] * 4
        self.state =  0
        self.sendMsgFcn = sendMsgFcn
        self.queryTimers = [MyTimer(1, partial(self.query,cmd)) for cmd in
            [MSG_GET_MOTOR_ERROR,
             MSG_GET_ENCODER_ERROR,
             MSG_GET_SENSORLESS_ERROR,
             MSG_GET_ENCODER_ESTIMATES,
             MSG_GET_ENCODER_COUNT,
             MSG_GET_IQ,
             MSG_GET_SENSORLESS_ESTIMATES,
             MSG_GET_VBUS_VOLTAGE]]

        
    def update(self):
        for timer in self.queryTimers:
            timer.update()

    # send message
    def query(self, cmd):
        for axis in range(4):
            self.sendMsgFcn(axis, cmd)
    def estop(self):
        self.query(MSG_ODRIVE_ESTOP)
        self.query(MSG_ODRIVE_ESTOP)
        self.query(MSG_ODRIVE_ESTOP)
    def clear_errors(self):
        self.query(MSG_CLEAR_ERRORS)
    def reboot(self, device: int):
        self.sendMsgFcn(device // 2, MSG_RESET_ODRIVE)
    def set_state(self, axis, state):
        self.sendMsgFcn(axis, MSG_SET_AXIS_REQUESTED_STATE, state)
    def set_ctrl_mode(self, axis, mode):
        self.sendMsgFcn(axis, MSG_SET_CONTROLLER_MODES, [mode, 1]) # always passthrough
        self.axes[axis].control_mode_believed = mode
    def set_setpoint(self, axis, setpoint):
        mode = self.axes[axis].control_mode_believed
        if mode == 1:
            self.sendMsgFcn(axis, MSG_SET_INPUT_TORQUE, [setpoint])
        elif mode == 2:
            self.sendMsgFcn(axis, MSG_SET_INPUT_VEL, [setpoint, 0])
        elif mode == 3:
            self.sendMsgFcn(axis, MSG_SET_INPUT_POS, [setpoint, 0, 0])
        else:
            print('unknown mode during setpoint command: {}'.format(mode))
    def set_calib_length(self, axis, other_axis):
        self.axes[axis].calib_lengths[other_axis] = self.axes[axis].pos
        return True
    def export_calibration(self):
        data = {'calib_lengths':
                {i: axis.calib_lengths for i, axis in enumerate(self.axes)}
            }
        datajson = json.dumps(data)
        fname = 'calib/' + datetime.datetime.now().strftime('%Y%m%d_%H%M') + '.json'
        with open(fname, 'w') as f:
            f.write(datajson)
        print(data)
        return tuple(axis.calib_lengths for axis in self.axes)

    # receive message
    def callback(self, node, cmd, data: bytes):
        # unpack everything at first
        f1, f2 = struct.unpack('<2f', data)  # little endian, 2 floats
        i1, i2 = struct.unpack('<2i', data)  # little endian, 2 ints (4 bytes)
        p, v, t = struct.unpack('<fhh', data)  # little endian, float-short-short
        if (node > 3):
            print('\tinvalid node! - {} {} {}'.format(node, cmd, data.hex()))
            return
        if (cmd == MSG_ODRIVE_HEARTBEAT):
            if i2 <= AxisStateEnum.AXIS_STATE_HOMING.value and i2 >= 0:
                self.axes[node].errorcode = i1
                self.axes[node].state = i2
                self.axes[node].last_hb_t = time.time()
        elif (cmd in [MSG_ODRIVE_ESTOP,
                    MSG_START_ANTICOGGING,
                    MSG_RESET_ODRIVE,
                    MSG_CLEAR_ERRORS]):
            print('\tThis messastatege should not be coming from the slave')
        # query info
        elif (cmd == MSG_GET_MOTOR_ERROR):
            self.axes[node].motorError = i1
        elif (cmd == MSG_GET_ENCODER_ERROR):
            self.axes[node].encoderError = i1
        elif (cmd == MSG_GET_SENSORLESS_ERROR):
            self.axes[node].sensorlessError = i1
        elif (cmd == MSG_GET_ENCODER_ESTIMATES):
            self.axes[node].pos = f1
            self.axes[node].vel = f2
        elif (cmd == MSG_GET_ENCODER_COUNT):
            self.axes[node].encoderCount = (i1, i2)
        elif (cmd == MSG_GET_IQ):
            self.axes[node].current_setpoint = f1
            self.axes[node].current_measured = f2
        elif (cmd == MSG_GET_SENSORLESS_ESTIMATES):
            self.axes[node].sensorlessEst = (f1, f2)
        elif (cmd == MSG_GET_VBUS_VOLTAGE):
            self.axes[node].voltage = f1
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

class RobotState(Enum):
    IDLE = 1
    ZEROING = 2
    CALIBRATING = 3

class AxisState:
    def __init__(self):
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
        self.control_mode_believed = 1
        self.calib_lengths = [float('nan'),]*4
