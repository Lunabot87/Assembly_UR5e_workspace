#!/usr/bin/env python
import tf
import rospy
import time
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander
from ur5e_inv_kin_wrapper import UR5eInvKinWrapper

class MoveGroupCommanderWrapper(MoveGroupCommander):
	def __init__(self, group_name):
		# super(MoveGroupCommanderWrapper, self).__init__(group_name)
		MoveGroupCommander.__init__(self, group_name)
		
		self.ur5e = UR5eInvKinWrapper()
		self.listener = tf.TransformListener()

	def get_name(self):
		'''
		how to handle method overloading
		'''
		print "My name is {}".format(self.__class__.__name__)
		# return super(MoveGroupCommanderWrapper, self).get_name()
		return MoveGroupCommander.get_name(self)

	def _array_to_js(self, array):
		js = JointState()
		js.position = array
		return js

	def move_to_pose_goal(self, pose_goal):
		current_joint = self.get_current_joint_values()

		(trans, rot) = self.listener.lookupTransform('/rob1_real_base_link', '/rob1_real_ee_link', rospy.Time(0))
		trans[0] += 0.05
		trans[2] += 0.05

		joints = self.ur5e.solve_and_sort(trans, rot, current_joint)
		js = self._array_to_js(joints[:, 0])
		traj = self.plan(js) # returns tuple (bool, JointTrajectory, float, MoveItErrorCodes)
		
		return traj

# def main():
# 	pass

# if __name__ == '__main__':
# 	main()


rospy.init_node('move_group_wrapper_test', anonymous=True)
mg_rob1 = MoveGroupCommanderWrapper('rob1_arm')
# mg.move_to_pose_goal(None)

stefan = Parts('chair_part1')
