#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
import moveit_commander
import moveit_msgs.msg
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from tf.transformations import *
from tf import *


from Assembly_Urx import UrxMotion
# import Assembly_Urx


def euler2Pose(euler):
    euler2Pose = Pose()
    qut = quaternion_from_euler(euler[3],euler[4],euler[5])
    euler2Pose.position.x = euler[0]
    euler2Pose.position.y = euler[1]
    euler2Pose.position.z = euler[2]
    euler2Pose.orientation.x = qut[0]
    euler2Pose.orientation.y = qut[1]
    euler2Pose.orientation.z = qut[2]
    euler2Pose.orientation.w = qut[3]

    return euler2Pose


def main():

    CLOSE = 255
    OPEN = 0

    rospy.init_node('Assembly_example', anonymous=True)

    pub = rospy.Publisher('/camera_op', Float32, queue_size=10)

    mg_rob1 = moveit_commander.MoveGroupCommander("rob1_arm")
    #mg_rob2 = moveit_commander.MoveGroupCommander("rob2_arm")
    
    urx_rob1 = UrxMotion("192.168.13.101")
    #urx_rob2 = UrxMotion("192.168.13.100")

    listener = TransformListener()



    rob1_pin1_pre_grasp = mg_rob1.get_named_target_values("rob1_pin1_pre_grasp")
    rob1_pin1_grasp = mg_rob1.get_named_target_values("rob1_pin1_grasp")

    rob1_pin1_camera_check = mg_rob1.get_named_target_values("rob1_pin1_camera_check")


    # move rob1 to rob1_pin1_pre_grasp
    rob1_plan = mg_rob1.plan(rob1_pin1_pre_grasp)
    print "go?" 
    # raw_input()
    mg_rob1.execute(rob1_plan)

    # move rob1 to rob1_pin1_grasp
    rob1_plan = mg_rob1.plan(rob1_pin1_grasp)
    print "go?" 
    # raw_input()
    mg_rob1.execute(rob1_plan)

    # rob1 close gripper
    urx_rob1.gripper_move_and_wait(CLOSE)

    # move rob1 to rob1_pin1_pre_grasp
    rob1_plan = mg_rob1.plan(rob1_pin1_pre_grasp)
    print "go?" 
    # raw_input()
    mg_rob1.execute(rob1_plan)

    # check camera_pose
    rob1_plan = mg_rob1.plan(rob1_pin1_camera_check)
    print "go?" 
    # raw_input()
    mg_rob1.execute(rob1_plan)


    print "pub"
    # raw_input()

    data = Float32()
    data.data = 1
    pub.publish(data)


    print "run!!!!"
    # raw_input()

    time.sleep(2)

    # xyz, rpy = listener.lookupTransform('/world', '/circle_1', rospy.Time(0))

    # xyz[2] = 1.2

    # xyz.append(3.1415)
    # xyz.append(0)
    # xyz.append(0)

    xyz = [0.04103064042430966, 0.20883629567663087, 1.2, 3.1415, 0, 0]


    Pose = euler2Pose(xyz)

    
    mg_rob1.set_pose_target(Pose)
    rob1_plan = mg_rob1.plan()
    print "go?" 
    # raw_input()
    mg_rob1.execute(rob1_plan)

    print "go"

    
    # start_pose = urx_rob1.robot.getl()
    # time.sleep(0.2)
    # print start_pose
    print "========="
    print "spiral?"
    # raw_input()
    
    urx_rob1.reset()
    urx_rob1.spiral_motion()



if __name__ == '__main__':
    main()