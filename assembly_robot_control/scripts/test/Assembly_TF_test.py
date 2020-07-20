#!/usr/bin/env python

import rospy
from tf import *
from geometry_msgs.msg import *
from math import pi

class TF_test():

	def __init__(self):
		rospy.init_node('TF_update_node', anonymous=True)
		rospy.Subscriber("TF_update", TransformStamped, self.TF_update)
		rospy.Timer(rospy.Duration(0.01), self.sendTF_objectCB)

		self.TFlist = []
		self.TF_name_list = ['circle_1', 'circle_2', 'circle_3']
		self.br = TransformBroadcaster()

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