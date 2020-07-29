#-*- coding:utf-8 -*-

import Assembly_Process

class Assembly_mode():
	def __init__(self):
		self.pr = Assembly_process()

	def insert_pin(self, asm_msg):
		# (일단은) 모두 rob1 이 작업 
		# real_insert_target_pose = self.process.fine_tune_insert_target(asm_msg.parent.target) # pin일 때는 parent 타겟이 항상 하나
		self.pr.grab_pin("pin_name")
		self.pr.hand_over_pin_check("target_1")
		# 학부 보조 연구원
		self.pr.insert_spiral_motion(real_insert_target_pose)

	def insert_part(self, asm_msg):
		# rob1, rob2 작업, rob1이 작업 중심
		sorted_insert_target_poses = self.pr.sort_insert_target(asm_msg.parents)
		## move() - parent part는 고정, child part가 rob1이 작업을 할 수 있는 위치에 없으면 할수 있는 위치로 옮긴다
		is_moved = self.pr.hand_over_part(sorted_insert_target_poses, asm_msg)
		self.pr.grab_part(asm_msg.child, is_moved)
		self.pr.insert_part_motion(sorted_insert_target_poses[0])