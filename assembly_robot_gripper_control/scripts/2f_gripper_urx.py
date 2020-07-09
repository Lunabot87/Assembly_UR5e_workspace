import sys
import urx
import time
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

if __name__ == '__main__':
    rob = urx.Robot("192.168.13.101")
    robotiqgrip = Robotiq_Two_Finger_Gripper(rob)

    robotiqgrip.open_gripper()
    time.sleep(2)
    robotiqgrip.close_gripper()
    time.sleep(2)
    robotiqgrip.open_gripper()


    rob.close()
    print "true"
    sys.exit()