#!/usr/bin/env python
import rospy
import time
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

import tf.transformations as tf
from geometry_msgs.msg import TransformStamped

rospy.init_node("tf_test")
br = StaticTransformBroadcaster()
#br = TransformBroadcaster()

t = TransformStamped()
time.sleep(2)
t.header.stamp = rospy.Time.now()
t.header.frame_id = "table"
t.child_frame_id = "tf_test22"
t.transform.translation.x = 0
t.transform.translation.y = 0
t.transform.translation.z = 1
t.transform.rotation.x = 0
t.transform.rotation.y = 0
t.transform.rotation.z = 0
t.transform.rotation.w = 1
print(rospy.Time.now())
print(t)
br.sendTransform(t)
#rospy.spin()