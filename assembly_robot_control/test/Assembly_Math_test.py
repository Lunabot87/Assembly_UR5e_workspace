#!/usr/bin/env python
#-*- coding:utf-8 -*-



from tf.transformations import *
from geometry_msgs.msg import *


    def euler2Pose(self, euler):
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