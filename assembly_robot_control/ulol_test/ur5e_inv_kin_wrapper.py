#!/usr/bin/env python
class UR5eInvKinWrapper():
	def __init__(self):
		self.robot = "1"

	def inv_kin(self, current_joint, pose_goal):
		'''
		return best solution
		'''
		current_joint[0] += 0.3
		return current_joint