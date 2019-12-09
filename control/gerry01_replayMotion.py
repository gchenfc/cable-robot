#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Empty, Bool, Int32, Float32, UInt8
from enum import Enum

last_pos = [0, 0]
ctrl_state = 0
trajectory = []
record_dur = 8 # seconds
rate = 30 # Hz

class State_t(Enum):
    IDLE = 0
    HOMING = 1
    RECORDING = 2
    REPLAYING = 3

def activate_motors(pub_activate0=None, pub_activate1=None, on=True): # not working
    if pub_activate0 is None:
        pub_activate0 = rospy.Publisher('/odrv/axis0/activate', Bool, queue_size=1)
    if pub_activate1 is None:
        pub_activate1 = rospy.Publisher('/odrv/axis1/activate', Bool, queue_size=1)
    pub_activate0.publish(on)
    pub_activate1.publish(on)
    return pub_activate0, pub_activate1

def init_command_motors():
    pub_home = rospy.Publisher('/cable_controller/set_state', UInt8, queue_size=1)
    pub_motor0pos = rospy.Publisher('/odrv/axis0/set_pos', Int32, queue_size=1)
    pub_motor1pos = rospy.Publisher('/odrv/axis1/set_pos', Int32, queue_size=1)
    pub_motor0cur = rospy.Publisher('/odrv/axis0/set_cur', Float32, queue_size=1)
    pub_motor1cur = rospy.Publisher('/odrv/axis1/set_cur', Float32, queue_size=1)
    return pub_motor0pos, pub_motor1pos, pub_motor0cur, pub_motor1cur, pub_home

def read_pos_0_cb(data):
    global last_pos
    last_pos[0] = data.data
def read_pos_1_cb(data):
    global last_pos
    last_pos[1] = data.data
def read_ctrl_state_cb(data):
    global ctrl_state
    ctrl_state = data.data

def command_motors(pub_motor0pos, pub_motor1pos, pos):
    pub_motor0pos.publish(pos[0])
    pub_motor1pos.publish(pos[1])
    print("\tcommanded motors to move to position "+str(pos))

def command_motors_cur(pub_motor0cur, pub_motor1cur, cur):
    pub_motor0cur.publish(cur[0])
    pub_motor1cur.publish(cur[1])
    print("\tcommanded motors to apply current "+str(cur))

def print_instructions():
    print('Commands:')
    print('   1 - Home')
    print('   2 - Record motion')
    print('   3 - Playback motion')
    print('   0 - exit')

def main():
    global last_pos, ctrl_state, trajectory
    print("starting")
    rospy.init_node('desktopcommander')
    r = rospy.Rate(rate)
    print("activating motors...")
    pub_activate0, pub_activate1 = activate_motors()
    print('setting up subscribers')
    rospy.Subscriber('/odrv/axis0/cur_pos', Float32, read_pos_0_cb)
    rospy.Subscriber('/odrv/axis1/cur_pos', Float32, read_pos_1_cb)
    rospy.Subscriber('/cable_controller/cur_state', UInt8, read_ctrl_state_cb)
    print('setting up publishers...')
    pub_motor0pos, pub_motor1pos, pub_motor0cur, pub_motor1cur, pub_home = init_command_motors()
    pub_clear_errors_0 = rospy.Publisher("/odrv/axis0/clear_errors", Empty, queue_size=1)
    pub_clear_errors_1 = rospy.Publisher("/odrv/axis1/clear_errors", Empty, queue_size=1)
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
                pub_clear_errors_0.publish()
                pub_clear_errors_1.publish()
                if (state == State_t.RECORDING):
                    trajectory = []
                    activate_motors(pub_activate0, pub_activate1)
                    command_motors_cur(pub_motor0cur, pub_motor1cur, [3, 3])
                if (state == State_t.REPLAYING):
                    activate_motors(pub_activate0, pub_activate1)
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
                trajectory.append((last_pos[0], last_pos[1]))
                index = index + 1
            else:
                index = 0
                print('finished recording.')
                # print(trajectory)
                command_motors_cur(pub_motor0cur, pub_motor1cur, [0, 0])
                activate_motors(pub_activate0, pub_activate1, on=False)
                state = State_t.IDLE
        elif state == State_t.REPLAYING:
            if index < len(trajectory):
                # print(trajectory[index])
                command_motors(pub_motor0pos, pub_motor1pos, trajectory[index])
                index = index + 1
            else:
                command_motors_cur(pub_motor0cur, pub_motor1cur, [0, 0])
                activate_motors(pub_activate0, pub_activate1, on=False)
                print('finished replaying')
                index = 0
                state = State_t.IDLE
        else:
            print("unknown state - returning to IDLE")
            state = State_t.IDLE
        r.sleep()

if __name__ == '__main__':
    main()