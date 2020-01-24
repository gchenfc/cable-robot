#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Empty, Bool, Int32, Float32, UInt8
from enum import Enum

NUM_MOTORS = 4

last_pos = [0,]*NUM_MOTORS
last_err = [0,]*NUM_MOTORS
ctrl_state = 0
trajectory = []
record_dur = 8 # seconds
rate = 30 # Hz

class State_t(Enum):
    IDLE = 0
    HOMING = 1
    RECORDING = 2
    REPLAYING = 3
    ERROR = 4
state = State_t.IDLE

def activate_motors(pub_activate=None, on=True): # not working
    if pub_activate is None:
        pub_activate = []
        for i in range(NUM_MOTORS):
            pub_activate.append(rospy.Publisher('/odrv/axis%d/activate'%(i), Bool, queue_size=1))
    for i in range(NUM_MOTORS):
        pub_activate[i].publish(on)
    return pub_activate

def init_command_motors():
    pub_home = rospy.Publisher('/cable_controller/set_state', UInt8, queue_size=1)
    pub_motorPos = []
    pub_motorCur = []
    for i in range(NUM_MOTORS):
        pub_motorPos.append(rospy.Publisher('/odrv/axis%d/set_pos'%(i), Int32, queue_size=1))
        pub_motorCur.append(rospy.Publisher('/odrv/axis%d/set_cur'%(i), Float32, queue_size=1))
    return pub_motorPos, pub_motorCur, pub_home

def read_pos_0_cb(data):
    global last_pos
    last_pos[0] = data.data
def read_pos_1_cb(data):
    global last_pos
    last_pos[1] = data.data
def read_pos_2_cb(data):
    global last_pos
    last_pos[2] = data.data
def read_pos_3_cb(data):
    global last_pos
    last_pos[3] = data.data
def read_err_cb(data, axis):
    if data.data != 0:
        global state, pub_clear_errors
        if (state != State_t.ERROR):
            state = State_t.ERROR
            for i in range(NUM_MOTORS):
                pub_clear_errors[i].publish()
        if (last_err[axis] == 0):
            print('Error detected in axis %d, switching to error state'%axis)
        last_err[axis] = data.data

def read_ctrl_state_cb(data):
    global ctrl_state
    ctrl_state = data.data

def command_motors(pub_motorPos, pos):
    for i in range(NUM_MOTORS):
        pub_motorPos[i].publish(pos[i])
    print("\tcommanded motors to move to position "+str(pos))

def command_motors_cur(pub_motorCur, cur):
    for i in range(NUM_MOTORS):
        pub_motorCur[i].publish(cur[i])
    print("\tcommanded motors to apply current "+str(cur))

def print_instructions():
    print('Commands:')
    print('   1 - Home')
    print('   2 - Record motion')
    print('   3 - Playback motion')
    print('   0 - exit')

def main():
    global last_pos, ctrl_state, trajectory, state, pub_clear_errors
    print("starting")
    rospy.init_node('desktopcommander')
    r = rospy.Rate(rate)
    print("activating motors...")
    pub_activate = activate_motors(on=False)
    print('setting up subscribers')
    rospy.Subscriber('/odrv/axis0/cur_pos', Float32, read_pos_0_cb)
    rospy.Subscriber('/odrv/axis1/cur_pos', Float32, read_pos_1_cb)
    rospy.Subscriber('/odrv/axis2/cur_pos', Float32, read_pos_2_cb)
    rospy.Subscriber('/odrv/axis3/cur_pos', Float32, read_pos_3_cb)
    for axis in range(4):
        rospy.Subscriber('/odrv/axis%d/error'%(axis), Int32, read_err_cb, callback_args=axis)
    rospy.Subscriber('/cable_controller/cur_state', UInt8, read_ctrl_state_cb)
    print('setting up publishers...')
    pub_motorPos, pub_motorCur, pub_home = init_command_motors()
    pub_clear_errors = []
    for i in range(NUM_MOTORS):
        pub_clear_errors.append(rospy.Publisher("/odrv/axis%d/clear_errors"%(i), Empty, queue_size=1))
    index = 0
    state = State_t.IDLE
    print("Ready!")
    while not rospy.is_shutdown():
        if state == State_t.IDLE:
            print_instructions()
            try:
                resp = input('please enter a command: ')
                if (resp == 0):
                    print('exiting')
                    rospy.signal_shutdown("user exit")
                    continue
                state = State_t(resp)
                index = 0
                for i in range(NUM_MOTORS):
                    pub_clear_errors[i].publish()
                if (state == State_t.RECORDING):
                    trajectory = []
                    activate_motors(pub_activate, True)
                    command_motors_cur(pub_motorCur, [4,]*NUM_MOTORS)
                if (state == State_t.REPLAYING):
                    activate_motors(pub_activate, True)
                if (state == State_t.HOMING):
                    pub_home.publish(1)
                    ctrl_state = 1
            except ValueError:
                print('invalid state '+str(resp))
        elif state == State_t.HOMING:
            if ctrl_state == 0:
                print('done homing!')
                state = State_t.IDLE
        elif state == State_t.RECORDING:
            if index < (record_dur * rate):
                trajectory.append(tuple(last_pos))
                index = index + 1
            else:
                index = 0
                print('finished recording.')
                # print(trajectory)
                # command_motors_cur(pub_motorCur, [0,]*NUM_MOTORS)
                activate_motors(pub_activate, on=False)
                state = State_t.IDLE
        elif state == State_t.REPLAYING:
            if index < len(trajectory):
                # print(trajectory[index])
                command_motors(pub_motorPos, trajectory[index])
                index = index + 1
            else:
                # command_motors_cur(pub_motorCur, [0,]*NUM_MOTORS)
                activate_motors(pub_activate, on=False)
                print('finished replaying')
                index = 0
                state = State_t.IDLE
        elif state == State_t.ERROR:
            activate_motors(pub_activate, on=False)
            for i in range(NUM_MOTORS):
                pub_clear_errors[i].publish()
                last_err[i] = 0
            print("sent clear_error messages")
            state = State_t.IDLE
        else:
            print("unknown state - returning to IDLE")
            state = State_t.IDLE
        r.sleep()

if __name__ == '__main__':
    main()