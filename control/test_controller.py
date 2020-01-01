'''Tests controller.py
Authors: Gerry Chen and Frank Dellaert
'''
import rospy
from controller import Controller
import numpy as np
import time

#TODO(gerry): make this unit-testable ???
def main():
    print("starting")
    
    rospy.init_node('desktopcommander', log_level=rospy.DEBUG)
    r = rospy.Rate(25)

    controller = Controller(4, 25)
    passed = True
    index = -1
    test_start_time = time.time()

    print("Ready!")

    while not rospy.is_shutdown():
        controller.update()
        # state transition
        if time.time() > (test_start_time + 1):
            if controller.get_state() is Controller.State_t.IDLE:
                if index >= 0:
                    if not passed:
                        print("Failed test %d"%(index))
                    else:
                        print("Passed test %d"%(index))
                index = index + 1
                if index == 0:
                    # controller.set_state(Controller.State_t.INIT)
                    pass
                elif index == 1:
                    # pass
                    controller.set_state(Controller.State_t.HOMING)
                elif index == 2:
                    controller.goto_pos(np.array([0.0, 0.0]))
                elif index == 3:
                    controller.goto_pos(np.array([0.5, 0.5]))
                elif index == 4:
                    controller.goto_pos(np.array([1.0, 1.0]))
                else:
                    controller.set_motors_activate([False,]*4)
                    rospy.signal_shutdown("Test Complete")
                test_start_time = time.time()
        # state checking
        if index == 0:
            passed = True
        elif index == 1:
            passed = True
        elif index == 2:
            passed = np.linalg.norm(controller.calc_pos() - np.array([0.0, 0.0])) < 0.1
        elif index == 3:
            passed = np.linalg.norm(controller.calc_pos() - np.array([0.5, 0.5])) < 0.1
        elif index == 4:
            passed = np.linalg.norm(controller.calc_pos() - np.array([1.0, 1.0])) < 0.1
        else:
            passed = True
        r.sleep()

if __name__ == '__main__':
    main()
