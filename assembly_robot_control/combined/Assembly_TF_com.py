#!/usr/bin/env python

import rospy
from tf import *
from geometry_msgs.msg import *
from math import pi

import moveit_commander
import moveit_msgs.msg

class TF_test():

	def __init__(self):
		rospy.init_node('TF_update_node', anonymous=True)
		rospy.Subscriber("TF_update", TransformStamped, self.TF_update)
		rospy.Timer(rospy.Duration(0.01), self.sendTF_objectCB)

		self.TFlist = []
		self.TF_name_list = ['circle_1', 'circle_2', 'circle_3']
		self.br = TransformBroadcaster()

		moveit_commander.roscpp_initialize(sys.argv)
		scene = moveit_commander.PlanningSceneInterface()

		item = geometry_msgs.msg.PoseStamped()
		item.header.frame_id = "world"

		item.pose.position.x = -0.318
		item.pose.position.y = 0.274
		item.pose.position.z = 1.0
		item.pose.orientation.x = 0
		item.pose.orientation.y = 0
		item.pose.orientation.z = 0
		item.pose.orientation.w = 1

		pose = [-0.318, 0.274, 0.003, 0, 0, 0]

		scene.add_mesh("part5", item , "/home/care/test_ws/src/tuto_bot/tuto_scripts/stl/chair part5.SLDPRT.STL")

	def main(self):
		rospy.spin()


	def TF_update(self, tfmessage):


		item = geometry_msgs.msg.PoseStamped()
		item.header.frame_id = tfmessage.header.frame_id

		item.pose.position.x = tfmessage.transform.translation.x
		item.pose.position.y = tfmessage.transform.translation.y
		item.pose.position.z = tfmessage.transform.translation.z
		item.pose.orientation.x = tfmessage.transform.rotation.x
		item.pose.orientation.y = tfmessage.transform.rotation.y
		item.pose.orientation.z = tfmessage.transform.rotation.z
		item.pose.orientation.w = tfmessage.transform.rotation.w

		self.TFlist.append(item)


	def sendTF_objectCB(self, TF):
		if self.TFlist:
			for pose,name in zip(self.TFlist,self.TF_name_list):
				self.br.sendTransform((pose.pose.position.x,pose.pose.position.y,pose.pose.position.z),
			                         (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
			                         rospy.Time.now(),
			                         name,
			                         pose.header.frame_id)



if __name__ == '__main__':
    aa = TF_test()
    aa.main()