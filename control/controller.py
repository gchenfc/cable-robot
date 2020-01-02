#!/usr/bin/env python
'''Provides useful utilities for controlling a cable robot
You must:
    initialize the ros node
    call rospy.spin() / update rospy
    call controller.update()
Example usage in main()
Authors: Gerry Chen and Frank Dellaert
'''
import rospy
import numpy as np
from std_msgs.msg import Empty, Bool, Int32, Float32, UInt8
from enum import Enum, unique
import time

class Controller:
    @unique
    class State_t(Enum):
        IDLE = 1
        INIT = 2 # taught cables
        HOMING = 3
        # COMMAND = 4
        GOTOPOS = 5
        RECORDING = 6
        REPLAYING = 7
        ERROR = 8
    @unique
    class Command_t(Enum):
        NONE = 0
        POS = 1
        VEL = 2
        CUR = 3

    def __init__(self, NUM_MOTORS=4, rate=30, CNTS_PER_REV=-8192):
        # robot configuration
        self.NUM_MOTORS = NUM_MOTORS
        self.CNTS_PER_REV = CNTS_PER_REV
        self.mount_config = [None,]*self.NUM_MOTORS

        # local copy of variables
        self._last_err = [0,]*self.NUM_MOTORS
        self._last_state = [0,]*self.NUM_MOTORS
        self._last_pos = [0,]*self.NUM_MOTORS
        self._last_vel = [0,]*self.NUM_MOTORS
        self._last_cur = [0,]*self.NUM_MOTORS
        self._last_setCur = [0,]*self.NUM_MOTORS
        self._last_volt = [0,]*self.NUM_MOTORS
        self._last_curLim = [0,]*self.NUM_MOTORS
        self._mcu_state = 0

        # control variables
        self._setpoint_pos = []
        self._trajectory = []
        self._traj_index = 0
        self._record_dur = 8 # seconds
        self._rate = rate # Hz
        self._state_begin_time = 0
        self._home = Home(self)
        self._command_timestamp = [0,]*self.NUM_MOTORS

        # publishers / subscribers
        self.setup_publishers()
        self.setup_subscribers()

        # init
        self._state = self.State_t.IDLE
        self._next_state = self.State_t.IDLE
        self._commands = [(self.Command_t.NONE, 0),]*self.NUM_MOTORS

    # ------------------------------------------ SETUP ------------------------------------------ #
    def setup_publishers(self):
        rospy.loginfo('setting up publishers...')
        self.pub_m_estop = []
        self.pub_m_activate = []
        self.pub_m_pos = []
        self.pub_m_vel = []
        self.pub_m_cur = []
        # self.pub_m_KpVel = [] # not yet tested
        # self.pub_m_KiVel = []
        self.pub_m_KpPos = []
        self.pub_m_clear_errors = []
        self.pub_m_curLim = []
        self.pub_m_pos_est = []
        self.pub_mcu_state = rospy.Publisher('/cable_controller/set_state', UInt8, queue_size=1)
        for i in range(self.NUM_MOTORS):
            self.pub_m_estop.append(    rospy.Publisher("/odrv/axis%d/estop"%(i),   Empty, queue_size=1))
            self.pub_m_activate.append( rospy.Publisher("/odrv/axis%d/activate"%(i), Bool, queue_size=1))
            self.pub_m_pos.append(      rospy.Publisher("/odrv/axis%d/set_pos"%(i), Int32, queue_size=1))
            self.pub_m_vel.append(      rospy.Publisher("/odrv/axis%d/set_vel"%(i), Float32, queue_size=1))
            self.pub_m_cur.append(      rospy.Publisher("/odrv/axis%d/set_cur"%(i), Float32, queue_size=1))
            # self.pub_m_KpVel.append(    rospy.Publisher("/odrv/axis%d/set_KpVel"%(i), Float32, queue_size=1))
            # self.pub_m_KiVel.append(    rospy.Publisher("/odrv/axis%d/set_KiVel"%(i), Float32, queue_size=1))
            self.pub_m_KpPos.append(    rospy.Publisher("/odrv/axis%d/set_KpPos"%(i), Float32, queue_size=1))
            self.pub_m_clear_errors.append(  rospy.Publisher("/odrv/axis%d/clear_errors"%(i), Empty, queue_size=1))
            self.pub_m_curLim.append(   rospy.Publisher("/odrv/axis%d/set_lim_cur"%(i), Float32, queue_size=1))
            self.pub_m_pos_est.append(  rospy.Publisher("/odrv/axis%d/set_pos_est"%(i), Int32, queue_size=1))
    
    def setup_subscribers(self):
        rospy.loginfo('setting up subscribers')
        for axis in range(self.NUM_MOTORS):
            rospy.Subscriber("/odrv/axis%d/error"%(axis), Int32, self.read_err_cb, callback_args=axis)
            rospy.Subscriber("/odrv/axis%d/cur_state"%(axis), Int32, self.read_state_cb, callback_args=axis)
            rospy.Subscriber("/odrv/axis%d/cur_pos"%(axis), Float32, self.read_pos_cb, callback_args=axis)
            rospy.Subscriber("/odrv/axis%d/cur_vel"%(axis), Float32, self.read_vel_cb, callback_args=axis)
            rospy.Subscriber("/odrv/axis%d/cur_cur"%(axis), Float32, self.read_cur_cb, callback_args=axis)
            rospy.Subscriber("/odrv/axis%d/cur_setCur"%(axis), Float32, self.read_setCur_cb, callback_args=axis)
            rospy.Subscriber("/odrv/axis%d/cur_volt"%(axis), Float32, self.read_volt_cb, callback_args=axis)
            rospy.Subscriber("/odrv/axis%d/lim_cur"%(axis), Float32, self.read_cur_cb, callback_args=axis)
        rospy.Subscriber("/cable_controller/cur_state", UInt8, self.read_mcu_state_cb)

    # ---------------------------------------- CALLBACKS ---------------------------------------- #
    def read_err_cb(self, data, axis):
        if data.data != 0:
            if (self._state is not self.State_t.ERROR): # this needs to be fixed
                # self._state = self.State_t.ERROR
                self.set_motors_clear_errors()
            # if (self._last_err[axis] == 0):
            #     print('Error detected in axis %d, switching to error state'%axis)
            # self._last_err[axis] = data.data
    def read_state_cb(self, data, axis):
        self._last_state[axis] = data.data
    def read_pos_cb(self, data, axis):
        self._last_pos[axis] = data.data
    def read_vel_cb(self, data, axis):
        self._last_vel[axis] = data.data
    def read_cur_cb(self, data, axis):
        self._last_cur[axis] = data.data if self._last_state[axis] == 8 else 0
    def read_setCur_cb(self, data, axis):
        self._last_setCur[axis] = data.data
    def read_volt_cb(self, data, axis):
        self._last_volt[axis] = data.data
    def read_curLim_cb(self, data, axis):
        self._last_curLim[axis] = data.data
    def read_mcu_state_cb(self, data):
        self._mcu_state = data.data

    # -------------------------------------- CALCULATIONS -------------------------------------- #
    # TODO(gerry): different naming convention for lengths and positions
    def calc_pos(self):
        return self.fk(self._last_pos)
    def fk(self, lengths):
        # TODO
        return np.array([0, 0])
        def feasible(pos, lengths):
            for i in range(self.NUM_MOTORS):
                if np.linalg.norm(pos, self.mount_config[i]) > lengths[i]:
                    return False
            return True
    def ik(self, pos):
        # TODO
        return np.array([0,]*self.NUM_MOTORS)

    # ----------------------------------------- ACTIONS ----------------------------------------- #
    # 'set' functions immediately publish the given parameter
    def set_mcu_state(self, state):
        self.pub_mcu_state.publish(state)
        self._mcu_state = state
    def set_motors_estop(self):
        self.set_motors(self.pub_m_estop, None)
    def set_motors_activate(self, on=None): # not working
        if on is None:
            on = [True,]*self.NUM_MOTORS
        self.set_motors(self.pub_m_activate, on)
    def set_motors_pos(self, pos):
        self.set_motors(self.pub_m_pos, pos)
    def set_motors_vel(self, vel):
        self.set_motors(self.pub_m_vel, vel)
    def set_motors_cur(self, cur):
        self.set_motors(self.pub_m_cur, cur)
    # def set_motors_KpVel(KpVel):
    #     self.set_motors(self.pub_m_KpVel, KpVel)
    # def set_motors_KiVel(KiVel):
    #     self.set_motors(self.pub_m_KiVel, KiVel)
    def set_motors_KpPos(self, KpPos):
        self.set_motors(self.pub_m_KpPos, KpPos)
    def set_motors_clear_errors(self):
        self.set_motors(self.pub_m_clear_errors, None)
    def set_motors_curLim(self, curLim):
        self.set_motors(self.pub_m_curLim, curLim)
    def set_motors_pos_est(self, pos_est):
        self.set_motors(self.pub_m_pos_est, pos_est)

    def set_motors(self, publishers, datas=None):
        for i in range(self.NUM_MOTORS):
            if datas is None:
                publishers[i].publish()
            elif datas[i] is not None:
                publishers[i].publish(datas[i])

    # 'command' functions enter command state and controller does some state monitoring
    def command_motors_pos(self, pos):
        self.command_motors(self.pub_m_pos, pos, self.Command_t.POS)
        rospy.logdebug("\tcommanded motors to position "+str(pos))
    def command_motors_vel(self, vel):
        self.command_motors(self.pub_m_vel, vel, self.Command_t.VEL)
        rospy.logdebug("\tcommanded motors to velocity "+str(vel))
    def command_motors_cur(self, cur):
        self.command_motors(self.pub_m_cur, cur, self.Command_t.CUR)
        rospy.logdebug("\tcommanded motors to current "+str(cur))

    '''command_check: Checks if a command is already being executed properly
    Arguments:
        command:    tuple of (Command_t, setpoint)
        axis:       axis to apply command to
    Returns:
        check:      true if command already executed, false otherwise
    '''
    def command_check(self, command, axis):
        if command != self._commands[axis]: # new command
            self._command_timestamp[axis] = time.time()
            return False
        else: # old command
            if time.time() < (self._command_timestamp[axis] + 0.1):
                return True
            else:
                self._command_timestamp[axis] = self._command_timestamp[axis] + 0.1
                return False

    '''command_motor: Executes a single command to a single axis.  Must be in `COMMAND` state.
    Arguments:
        command:    tuple of (Command_t, setpoint)
        axis:       axis to apply command to
    '''
    def command_motor(self, command, axis):
        if self.command_check(command, axis):
            return
        command_type, setpoint = command
        if command_type is self.Command_t.NONE:
            self.pub_m_activate[axis].publish(False)
        else:
            if self._last_state[axis] != 8:
                self.pub_m_activate[axis].publish(True)
            if command_type is self.Command_t.POS:
                self.pub_m_pos[axis].publish(setpoint)
            elif command_type is self.Command_t.VEL:
                self.pub_m_vel[axis].publish(setpoint)
            elif command_type is self.Command_t.CUR:
                self.pub_m_cur[axis].publish(setpoint)
        self._commands[axis] = command

    '''command_motor: Executes commands to each axis
    Arguments:
        command:    list of tuples of (Command_t, setpoint)
    '''
    def command_motors(self, commands):
        for i, command in enumerate(commands):
            self.command_motor(command, i)

    # 'goto' functions go to setpoints in task space
    def goto_pos(self, pos):
        self._setpoint_pos = pos
        self.set_state(self.State_t.GOTOPOS)
    
    # --------------------------------------- CONTROLLER --------------------------------------- #
    def set_state(self, state):
        self._next_state = state
    def get_state(self):
        return self._state
    def _advance_state(self, next_state=None):
        # for i in range(NUM_MOTORS):
        #     pub_clear_errors[i].publish()
        if next_state is None:
            self._state = self._next_state
            self._next_state = self.State_t.IDLE
        else:
            self._state = next_state
        if self._state is self.State_t.IDLE:
            pass
        elif self._state is self.State_t.INIT:
            rospy.loginfo("Initializing...")
            self.set_motors_activate()
            self.set_motors_vel([-2*self.CNTS_PER_REV,]*self.NUM_MOTORS)
        elif self._state is self.State_t.HOMING:
            rospy.loginfo("Homing...")
            self._home.setup()
        elif self._state is self.State_t.GOTOPOS:
            pass
        elif self._state is self.State_t.RECORDING:
            self._trajectory = []
            self._traj_index = 0
            self.set_motors_activate([True,]*self.NUM_MOTORS)
            self.set_motors_cur([4,]*self.NUM_MOTORS)
        elif self._state is self.State_t.REPLAYING:
            self.set_motors_activate([True,]*self.NUM_MOTORS)
        self._state_begin_time = time.time()

    def update(self):
        if self._state is self.State_t.IDLE:
            if self._next_state is not self.State_t.IDLE:
                self._advance_state()
            elif time.time() > (self._state_begin_time + 1): # periodically reassert motors to idle
                self.set_motors_activate(on=[False,]*self.NUM_MOTORS)
                self._state_begin_time = time.time()
        elif self._state is self.State_t.INIT:
            rospy.logdebug(self._last_cur)
            if (time.time() > (self._state_begin_time + 0.5)):
                # start checking if moving
                for i in range(self.NUM_MOTORS):
                    if self._last_cur[i] < 5:
                        self._advance_state(self.State_t.INIT) # reissue velocity commands
                        self._state_begin_time = time.time() - 0.25
                        return
                    # if self._last_state is not 8: # closed loop control
                    #     return
                self.set_motors_activate(on=[False,]*self.NUM_MOTORS)
                self._advance_state()
        elif self._state is self.State_t.HOMING:
            # TODO
            if self._home.update():
                rospy.loginfo('done homing!')
                self.mount_config = self._home.mount_config
                self._advance_state()
        elif self._state is self.State_t.RECORDING:
            if self._traj_index < (record_dur * rate):
                trajectory.append(tuple(last_pos))
                self._traj_index = self._traj_index + 1
            else:
                self._traj_index = 0
                rospy.loginfo('finished recording.')
                # rospy.logdebug(trajectory)
                # command_motors_cur(pub_m_cur, [0,]*NUM_MOTORS)
                self.set_motors_activate(on=[False,]*self.NUM_MOTORS)
                self._advance_state()
        elif self._state is self.State_t.REPLAYING:
            if self._traj_index < len(trajectory):
                # rospy.logdebug(trajectory[self._traj_index])
                command_motors_pos(pub_m_pos, trajectory[self._traj_index])
                self._traj_index = self._traj_index + 1
            else:
                # command_motors_cur(pub_m_cur, [0,]*NUM_MOTORS)
                self.set_motors_activate(on=[False,]*self.NUM_MOTORS)
                rospy.loginfo('finished replaying')
                self._traj_index = 0
                self._advance_state()
        elif self._state is self.State_t.ERROR:
            self.set_motors_activate(on=[False,]*self.NUM_MOTORS)
            self.set_motors_clear_errors()
            for i in range(self.NUM_MOTORS):
                self._last_err[i] = 0
            rospy.logdebug("sent clear_error messages")
            self._advance_state()
        else:
            rospy.loginfo("unknown state %s - returning to IDLE"%(str(self._state)))
            self._advance_state()
        self.command_motors(self._commands)

class Home:
    @unique
    class State_t(Enum):
        INIT = 1
        # TIGHTENING = 2
        MOVING = 3
        ZEROING = 4
        RETURNING = 5
        ATLIM_WAIT = 6
        ATLIM_WIGGLE = 7
        DONE = 8
    
    def __init__(self, controller):
        self.controller = controller
        self.mount_config = None
        self.wiggle_prior = [None,]*(self.controller.NUM_MOTORS + 1)

    def setup(self):
        self.state = self.State_t.INIT
        self.axis = 0
        # self.wiggle_axis = 1
        # self.wiggle_set = 0
        # self.wiggle_start = 0
        # self.wait_timer = 0
        # self.wiggle_all = True
        self._state_begin_time = time.time()
    
    def start_wiggle(self):
        self.wiggle_start = self.controller._last_pos[axis]
        self.wiggle_set = self.controller._last_pos[self.wiggle_axis] + 3*self.controller.CNTS_PER_REV
        self.controller.command_motor((Controller.Command_t.POS, self.wiggle_set), self.wiggle_axis)
        self.wait_timer = time.time()

    def move(self, awaiting=False):
        commands = [(Controller.Command_t.NONE, 0),]*4
        v = 3*self.controller.CNTS_PER_REV
        v2 = 2.5*self.controller.CNTS_PER_REV # make the other axes a little slower to maintain tension

        if (not awaiting) and (abs(self.controller._last_cur[self.axis]) > 3): # let the other axes help out a little
            if self.axis == 0:
                pass
            elif self.axis == 1:
                # commands[0] = (Controller.Command_t.VEL, +v2)
                pass
            elif self.axis == 2:
                x = self.controller._last_pos[0] / self.controller.CNTS_PER_REV
                L = np.sqrt(np.square(x) + np.square(self.mount_config[1][0]))
                # commands[0] = (Controller.Command_t.VEL, +v2)
                # commands[1] = (Controller.Command_t.VEL, +v2 * x/L)
                l = self.controller._last_pos[1] / self.controller.CNTS_PER_REV
                print(L, l, L - l)
                commands[1] = (Controller.Command_t.VEL, (L - l)*self.controller.CNTS_PER_REV)
            elif self.axis == 3:
                X = self.mount_config[1][0]
                Y = self.mount_config[2][1]
                L = np.sqrt(np.square(X) + np.square(Y))
                L1 = X*X/L # distance from origin to point where cable 1 intersects cables 0/2 at a right angle
                l = self.controller._last_pos[0] / self.controller.CNTS_PER_REV # distance from origin to EE
                # commands[0] = (Controller.Command_t.VEL, +v2)
                x = l*X/L
                y = l*Y/L
                s1 = np.sqrt(np.square(y) + np.square(x-X))
                s2 = np.sqrt(np.square(x) + np.square(y-Y))
                print(s1, s2)
                l1 = self.controller._last_pos[1] / self.controller.CNTS_PER_REV
                l2 = self.controller._last_pos[2] / self.controller.CNTS_PER_REV
                commands[1] = (Controller.Command_t.VEL, (s1 - l1)*self.controller.CNTS_PER_REV)
                commands[2] = (Controller.Command_t.VEL, (s2 - l2)*self.controller.CNTS_PER_REV)
                # commands[1] = (Controller.Command_t.VEL, +v2 * (l-L1)/self.controller._last_pos[1])
                # commands[2] = (Controller.Command_t.VEL, +v2 * (l - (L-L1))/self.controller._last_pos[2])
        else:
            if (not awaiting):
                commands = [(Controller.Command_t.CUR, 2),]*4
            else:
                commands = [(Controller.Command_t.NONE, 0),]*4

        commands[self.axis] = (Controller.Command_t.VEL, -v)
        self.controller.command_motors(commands)
    
    def move_return(self):
        commands = [(Controller.Command_t.NONE, 0),]*4
        if self.axis > 3:
            self.controller.command_motors([(Controller.Command_t.NONE, 0),]*4)
            self.state = self.State_t.DONE
            return
        # return to origin
        if self.axis > 2:
            pass
            # if (self.controller._last_pos[2]/self.controller.CNTS_PER_REV) < \
            #    (self.mount_config[2][1]/self.controller.CNTS_PER_REV):
            #     commands[2] = (Controller.Command_t.VEL, 4*self.controller.CNTS_PER_REV)
            # else:
            #     commands[2] = (Controller.Command_t.POS, self.mount_config[2][1]*0.99)
        if self.axis > 1:
            if (self.controller._last_pos[2]/self.controller.CNTS_PER_REV) < \
               (self.mount_config[1][0]):
                if self.axis == 1:
                    pass
                    # commands[1] = (Controller.Command_t.VEL, 4*self.controller.CNTS_PER_REV)
                else:
                    commands[1] = (Controller.Command_t.VEL, -4*self.controller.CNTS_PER_REV)
            else:
                commands[1] = (Controller.Command_t.POS, self.mount_config[1][0]*0.99*self.controller.CNTS_PER_REV)
        if self.axis > 0:
            commands[0] = (Controller.Command_t.POS, 0)
        self.controller.command_motors(commands)

    def finished_axis(self):
        if self.state is not self.State_t.ZEROING: # zero axis
            self.controller.pub_m_pos_est[self.axis].publish(0)
            self._state_begin_time = time.time()
            self.state = self.State_t.ZEROING
        else:
            if abs(self.controller._last_pos[self.axis] / self.controller.CNTS_PER_REV) > 0.5:
                if time.time() > (self._state_begin_time + 0.5): # periodically resend zero signal
                    self.controller.pub_m_pos_est[self.axis].publish(0)
                    self._state_begin_time = time.time()
            else:
                # axis zero-ed, save mount_config
                if self.axis == 0:
                    self.mount_config = [(0,0),] * self.controller.NUM_MOTORS
                elif self.axis == 1:
                    self.mount_config[1] = (self.controller._last_pos[0]/self.controller.CNTS_PER_REV, 0)
                elif self.axis == 2:
                    self.mount_config[2] = (0, self.controller._last_pos[0]/self.controller.CNTS_PER_REV)
                elif self.axis == 3:
                    self.mount_config[3] = (self.controller._last_pos[2]/self.controller.CNTS_PER_REV,
                                            self.controller._last_pos[1]/self.controller.CNTS_PER_REV)
                    if 5 < abs(np.linalg.norm(self.mount_config[3]) -
                                self.controller._last_pos[0]/self.controller.CNTS_PER_REV):
                        rospy.logwarn('homing not consistent:%s\t%s'%(str(self.mount_config),
                                                                        str(self.controller._last_pos)))
                print('Calibrated Axis %d'%(self.axis))
                print('\tCurrent position is: %s'%(str(self.controller._last_pos)))
                print('\tnew mount_config is: %s'%(str(self.mount_config)))
                
                # next axis
                self.axis = self.axis + 1
                self.state = self.State_t.RETURNING
                self._state_begin_time = time.time()

    def update(self):
        if self.controller.NUM_MOTORS != 4:
            rospy.logwarn("Currently I only support 4 motors in a planar rectangular configuration")
            return True
        
        print(self.state)

        if self.state is self.State_t.INIT:
            self.state = self.State_t.MOVING
        elif self.state is self.State_t.MOVING:
            self.move()
            if (self.controller._last_cur[self.axis] > 9): #TODO: make this more robust
                self._state_begin_time = time.time()
                self.state = self.State_t.ATLIM_WAIT
        elif self.state is self.State_t.ATLIM_WAIT:
            if (self.controller._last_cur[self.axis] > 9):
                self.move(awaiting=True)
                if time.time() > (self._state_begin_time + 1.5):
                    self.finished_axis()
                    # self.wiggle_all = True
                    # self.start_wiggle()
                    # self.state = self.State_t.ATLIM_WIGGLE
            else:
                self.state = self.State_t.MOVING
        elif self.state is self.State_t.RETURNING:
            self.move_return()
            if (self.controller._last_pos[0] / self.controller.CNTS_PER_REV) > 1:
                if time.time() > (self._state_begin_time + 15):
                    rospy.logerr("Homing procedure failed to return after axis %d"%(self.axis-1))
                    self.state = self.State_t.DONE
            else:
                print('Done returning: current position is %s'%(str(self.controller._last_pos)))
                self.state = self.State_t.MOVING
        # elif self.state is self.State_t.ATLIM_WIGGLE:
        #     dist_to_setpoint = ((self.wiggle_set - self.controller._last_pos[self.wiggle_axis]) /
        #                                     self.controller.CNTS_PER_REV)
        #     if (dist_to_setpoint < 1) or (time.time() > (self.wait_timer + 1)):
        #         # done wiggling
        #         dist_moved = (self.wiggle_start - self.controller._last_pos[self.axis]) / self.controller.CNTS_PER_REV
        #         if dist_moved > 1:
        #             # wiggle more
        #             self.wiggle_all = False
        #             self.start_wiggle
        #         else:
        #             # next wiggle axis
        #             self.controller.command_motor((Controller.Command_t.NONE, 0), self.wiggle_axis)
        #             self.wiggle_axis = self.wiggle_axis + 1
        #             if self.wiggle_axis == self.axis:
        #                 self.wiggle_axis = self.wiggle_axis + 1
        #             if self.wiggle_axis < self.controller.NUM_MOTORS:
        #                 self.wiggle_axis = 0
        #                 if self.wiggle_all: #done
        #                     aoeu
        #                 else: # go through and wiggle all again
        #                     self.wiggle_all = True
        #                     self.start_wiggle()
        elif self.state is self.State_t.ZEROING:
            self.finished_axis()
        elif self.state is self.State_t.DONE:
            self.controller.command_motors([(Controller.Command_t.NONE, 0),]*4)
            return True

        return False

def print_instructions():
    rospy.loginfo('Commands:')
    rospy.loginfo('   1 - Home')
    rospy.loginfo('   2 - Record motion')
    rospy.loginfo('   3 - Playback motion')
    rospy.loginfo('   0 - exit')

def main():
    print("starting")
    
    rospy.init_node('desktopcommander', log_level=rospy.DEBUG)
    r = rospy.Rate(25)

    controller = Controller(4, 25)
    passed = True
    index = -1
    test_start_time = time.time()

    def onshutdown():
        controller.set_motors_activate([False,]*4)
    rospy.on_shutdown(onshutdown)

    print("Ready!")
    
    while not rospy.is_shutdown():
        controller.update()
        # state transition
        if time.time() > (test_start_time + 1):
            if controller.get_state() is Controller.State_t.IDLE:
                index = index + 1
                if index == 0:
                    controller.set_state(Controller.State_t.HOMING)
                    pass
                else:
                    controller.set_motors_activate([False,]*4)
                    rospy.signal_shutdown("Test Complete")
                test_start_time = time.time()
        r.sleep()

    # Current position is: [-385919.0, -316878.0, -248976.0, 13.0]
    # new mount_config is: [(0, 0), (28.3603515625, 0), (0, 37.2548828125), (30.392578125, 38.681396484375)]

if __name__ == '__main__':
    main()