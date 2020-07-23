#!/usr/bin/env python
import rospy
from moveit_commander import MoveGroupCommander
from ur5e_inv_kin_wrapper import UR5eInvKinWrapper

class MoveGroupCommanderWrapper(MoveGroupCommander):
	def __init__(self, group_name):
		# super(MoveGroupCommanderWrapper, self).__init__(group_name)
		MoveGroupCommander.__init__(self, group_name)
		self.ur5e = UR5eInvKinWrapper()

	def get_name(self):
		'''
		how to handle method overloading
		'''
		print "My name is {}".format(self.__class__.__name__)
		# return super(MoveGroupCommanderWrapper, self).get_name()
		return MoveGroupCommander.get_name(self)

	def move_to_pose_goal(self, pose_goal):
		current_joint = self.get_current_joint_values()
		# selected_joint_goal = self.ur5e.inv_kin(current_joint, pose_goal)
		# selected_joint_goal = self.ur5e.inv_kin(current_joint, pose_goal, 0)
		joint_goal_dictionary = self.ur5e.inv_kin_all(current_joint, pose_goal)
		
		print selected_joint_goal
		return self.go(selected_joint_goal, wait=True)


# def main():
# 	pass

# if __name__ == '__main__':
# 	main()


rospy.init_node('move_group_wrapper_test', anonymous=True)
move_group = MoveGroupCommanderWrapper('rob1_arm')
print move_group.get_name()
print move_group.get_current_joint_values()