import urx
rob1 = urx.Robot("192.168.13.101", use_rt=True)
rob2 = urx.Robot("192.168.13.100", use_rt=True)

import rospy
import moveit_commander
rospy.init_node('Assembly_example', anonymous=True)
mg_rob1 = moveit_commander.MoveGroupCommander("rob1_arm")



from tf import *
listener = TransformListener()
listener.lookupTransform('/rob1_real_base_link', '/rob1_real_ee_link', rospy.Time(0))

import logging
urx_logger = logging.getLogger("urx")
urx_logger.setLevel(logging.DEBUG)
ursecmon_logger = logging.getLogger("ursecmon")
ursecmon_logger.setLevel(logging.DEBUG)
urrtmon_logger = logging.getLogger("URRTMonitor")
urrtmon_logger.setLevel(logging.DEBUG)