#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Empty, Bool, Int32, Float32

def activate_motors(): # note: this is not working for some reason
    pub_activate0 = rospy.Publisher('/odrv/axis0/activate', Bool, queue_size=1)
    pub_activate0.publish(True)
    pub_activate1 = rospy.Publisher('/odrv/axis1/activate', Bool, queue_size=1)
    pub_activate1.publish(True)

def init_command_motors():
    pub_motor0cur = rospy.Publisher('/odrv/axis0/set_cur', Float32, queue_size=1)
    pub_motor0pos = rospy.Publisher('/odrv/axis0/set_pos', Int32, queue_size=1)
    pub_motor1cur = rospy.Publisher('/odrv/axis1/set_cur', Float32, queue_size=1)
    pub_motor1pos = rospy.Publisher('/odrv/axis1/set_pos', Int32, queue_size=1)
    return pub_motor0cur, pub_motor0pos, pub_motor1cur, pub_motor1pos

def command_motors(pub_motor0cur, pub_motor0pos, pub_motor1cur, pub_motor1pos, index):
    if (np.sin(index) < 10): # bypass conditional
        pub_motor0cur.publish(5)
        motor1pos = 10000*(7 + 4*np.cos(index)) # 30000, 110000
        pub_motor1pos.publish(motor1pos)
        print("\tcommanded motor 1 to move to position {:.3f}".format(motor1pos))
    else:
        motor0pos = 10000*(10.82 - 3.68*np.cos(index)) # 145000, 71.400
        pub_motor0pos.publish(motor0pos)
        pub_motor1cur.publish(5)
        print("\tcommanded motor 0 to move to position {:.3f}".format(motor0pos))

def main():
    print("starting")
    rospy.init_node('cable_controller')
    r = rospy.Rate(50) # 10hz
    pub_motor0cur, pub_motor0pos, pub_motor1cur, pub_motor1pos = init_command_motors()
    index = 0
    print("commanding motors...")
    while not rospy.is_shutdown():
        if (index == 0):
            print("activating motors...")
            activate_motors()
        command_motors(pub_motor0cur, pub_motor0pos, pub_motor1cur, pub_motor1pos, index)
        index = index + 0.02
        r.sleep()

if __name__ == '__main__':
    main()