#-*- coding:utf-8 -*-

import Assembly_Process

class Assembly_mode():
	def __init__(self):
		self.process = Assembly_process()

	def insert_pin(self, asm_msg):
		# (일단은) 모두 rob1 이 작업 
		real_insert_target_pose = self.process.fine_tune_insert_target(asm_msg.parent.target) # pin일 때는 parent 타겟이 항상 하나
		self.process.grab_pin(asm_msg.child, False)
		self.process.insert_spiral_motion(real_insert_target_pose)

	def insert_part(self, asm_msg):
		# rob1, rob2 작업, rob1이 작업 중심
		sorted_insert_target_poses = self.process.sort_insert_target(asm_msg.parents)
		## move() - parent part는 고정, child part가 rob1이 작업을 할 수 있는 위치에 없으면 할수 있는 위치로 옮긴다
		is_moved = self.process.hand_over_part(sorted_insert_target_poses, asm_msg)
		self.process.grab_part(asm_msg.child, is_moved)
		self.process.insert_part_motion(sorted_insert_target_poses[0])