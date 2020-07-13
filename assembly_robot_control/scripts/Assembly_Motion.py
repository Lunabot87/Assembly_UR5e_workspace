#-*- coding:utf-8 -*-

import moveit_commander
import moveit_msgs.msg

class Assembly_motion():
	def __init__(self):
		self.group1 = moveit_commander.MoveGroupCommander("rob1")
		self.group2 = moveit_commander.MoveGroupCommander("rob2")
		self.rob1 = urx.Robot('''rob1_ip''')
		self.rob2 = urx.Robot('''rob2_ip''')

	def pick_up(self, grasp):
		self.group1.go(grasp.pre_grasp)
		self.group1.go(grasp.grasp)
		self.group1.go(grasp.post_grasp)

	def move_to(target_pose):
		print "move_to"
		#수정전 임시 출력 

	def sprial_motion():
		# spiral motion을 진행하면서 pin을 insert 하는 작업
		# force값을 받아서 pin이 insert가 되었는지 아닌지 확인
		# insert가 되면 모션 중지하고 True를 반환
		# motion을 끝까지 진행하였는데도 insert가 안되었으면 False를 반환
		return is_inserted