#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import time
# from tuto_msgs.msg import *
from geometry_msgs.msg import *
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import *
from tf import *
# from tuto_init import *

class TF_update_node(object):

	def __init__(self):
		super(TF_update_node, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
	    

		rospy.init_node('TF_update_node', anonymous=True)

		rospy.Subscriber("TF_update", TransformStamped, self.TF_update)
	    # rospy.Subscriber("part_poses", part_pose, self.part_pose_upateCB)
		rospy.Subscriber("TF_assembly", Step, self.TF_assembly)

		self.pub = rospy.Publisher('/TF_assembly', Step, queue_size=10)

		self.pub_command = rospy.Publisher('/tuto_command', Step, queue_size=10)


		scene = moveit_commander.PlanningSceneInterface()

		rospy.Timer(rospy.Duration(0.01), self.sendTF_objectCB)

		self.br = TransformBroadcaster()

		self.scene = scene

		self.TFlist = []
		self.TF_name_list = []


	def TF_assembly(self, asm):
		i = 0
		for data in asm.Step:
			if data.parent.name in STEFAN_PART:
				self.scene.add_mesh(data.parent.name, data.parent.pose, PATH+data.parent.name+".STL")
				self.TF_name_update(data.parent.name, data.parent.pose)
				self.TF_name_update("target"+str(i), data.parent.target)

			else:
				print "no parent parts"
			
			if data.child.pin is not '':
				self.scene.add_mesh(data.child.pin, data.child.pose, PATH+data.child.pin+".STL")
				self.TF_name_update(data.child.pin, data.child.pose)

			elif data.child.part is not '':
				self.scene.add_mesh(data.child.part, data.child.pose, PATH+data.child.part+".STL")
				self.TF_name_update(data.child.pin, data.child.pose)

			else:
				print "no child parts" 
			i += 1

	def TF_name_update(self, name, pose):
		if name in self.TF_name_list:
			del self.TFlist[self.TF_name_list.index(name)]
			del self.TF_name_list[self.TF_name_list.index(name)]
			self.TFlist.append(pose)
			self.TF_name_list.append(name)


		else:
			self.TFlist.append(pose)
			self.TF_name_list.append(name)


	def TF_update(self, tfmessage):

		if tfmessage.child_frame_id in STEFAN_PART:
			item = geometry_msgs.msg.PoseStamped()
			item.header.frame_id = tfmessage.header.frame_id

			item.pose.position.x = tfmessage.transform.translation.x
			item.pose.position.y = tfmessage.transform.translation.y
			item.pose.position.z = tfmessage.transform.translation.z
			item.pose.orientation.x = tfmessage.transform.rotation.x
			item.pose.orientation.y = tfmessage.transform.rotation.y
			item.pose.orientation.z = tfmessage.transform.rotation.z
			item.pose.orientation.w = tfmessage.transform.rotation.w

			item_name = tfmessage.child_frame_id

			self.scene.add_mesh(item_name, item, PATH+item_name+".STL")
		else:
			print "no parts"

		if tfmessage.child_frame_id in self.TF_name_list:
			del self.TFlist[self.TF_name_list.index(tfmessage.child_frame_id)]
			del self.TF_name_list[self.TF_name_list.index(tfmessage.child_frame_id)]
			self.TFlist.append(tfmessage)
			self.TF_name_list.append(tfmessage.child_frame_id)


		else:
			self.TFlist.append(tfmessage)
			self.TF_name_list.append(tfmessage.child_frame_id)


	def sendTF_objectCB(self, TF):
		if self.TFlist:
			for pose,name in zip(self.TFlist,self.TF_name_list):
				self.br.sendTransform((pose.pose.position.x,pose.pose.position.y,pose.pose.position.z),
			                         (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
			                         rospy.Time.now(),
			                         name,
			                         pose.header.frame_id)


	def make_asm_msg(self, mode, parent_name, parent_frame_id, parent_pose ,target_frame_id, target_pose, child_name, child_frame_id, child_pose):
		asm_step = ASM_task()
		asm_step.mode = mode

		asm_step.parent.name = parent_name
		asm_step.parent.pose.header.frame_id = parent_frame_id
		re_p_pose = self.euler2Pose(parent_pose)
		asm_step.parent.pose.pose = re_p_pose

		asm_step.parent.target.header.frame_id = target_frame_id
		re_t_pose = self.euler2Pose(target_pose)
		asm_step.parent.target.pose = re_t_pose

		asm_step.child.pin = child_name
		asm_step.child.part = ''
		asm_step.child.pose.header.frame_id = child_frame_id
		re_c_pose = self.euler2Pose(child_pose)
		asm_step.child.pose.pose = re_c_pose


		return asm_step
	def euler2Pose(self,euler):
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

	def main(self):
		test_tf = TransformStamped()
		qut = quaternion_from_euler(1.5708,3.1405,0)
		test_tf.header.frame_id = "real_ee_link"
		test_tf.child_frame_id = "camera1"
		test_tf.transform.translation.x = 0.06
		test_tf.transform.translation.y = -0.045
		test_tf.transform.translation.z = 0.05
		test_tf.transform.rotation.x = qut[0]
		test_tf.transform.rotation.y = qut[1]
		test_tf.transform.rotation.z = qut[2]
		test_tf.transform.rotation.w = qut[3]

		# self.TFlist.append(test_tf)
		# self.TF_name_list.append(test_tf.child_frame_id)

		time.sleep(0)

		test_asm = Step()
		test_asm.Step.append(self.make_asm_msg("insert", "chair part2.SLDPRT", "real_base_link", [-0.04832,-0.37475,0.0,0,0,0], "chair part2.SLDPRT", [0.118,-0.014,0.010,1.5707,0,3.1415], "122620_1.SLDPRT", "real_base_link", [-0.3278, -0.1927, 0.04,1.5707,0,1.5707]))
		#test_asm.Step.append(self.make_asm_msg("insert", "chair part2.SLDPRT", "real_base_link", [-0.04832,-0.37475,0.0,0,0,0], "chair part2.SLDPRT", [0.118,-0.014,0.010,1.5707,0,3.1415], "122620_1.SLDPRT", "real_base_link", [-0.3278, -0.1927, 0.04,1.5707,0,1.5707]))
		#test_asm.Step.append(self.make_asm_msg("insert", "chair part2.SLDPRT", "real_base_link", [-0.04832,-0.37475,0.0,0,0,0], "chair part2.SLDPRT", [-0.107,-0.010,0.010,1.5707,0,3.1415], "122620_2.SLDPRT", "real_base_link", [-0.3675, -0.1927, 0.04,1.5707,0,1.5707]))
		self.pub.publish(test_asm)
		time.sleep(0)
		self.pub_command.publish(test_asm)
		print test_asm
		rospy.spin()

def main():
	try:
		tutorial = TF_update_node()
		tutorial.main()
    	

		print "============ Python tutorial demo complete!"
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
	 	return

if __name__ == '__main__':
  main()
