#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from Assembly_Process_test import Assembly_process



class Assembly_mode():

	def __init__(self):
		rospy.init_node('Assembly_Mode', anonymous=True)
		self.pr = Assembly_process(rospy)

	def insert_pin(self, asm_msg):
		# (일단은) 모두 rob1 이 작업 
		# real_insert_target_pose = self.process.fine_tune_insert_target(asm_msg.parent.target) # pin일 때는 parent 타겟이 항상 하나
		self.pr.grab_pin(pin) #asm_msg.child.pin

		robot = self.pr.hand_over_pin_check(pin, target) #asm_msg.parent.target.name
		self.pr.hold_assist(robot, "chair_part2", target)
		self.pr.fine_tune_insert_target(target, robot)
		# 학부 보조 연구원
		self.pr.insert_spiral_motion(robot)
		self.pr.hold_assist_reset(robot, "chair_part2", target)



	def insert_pin_test(self, pin, target):
		# (일단은) 모두 rob1 이 작업 
		# real_insert_target_pose = self.process.fine_tune_insert_target(asm_msg.parent.target) # pin일 때는 parent 타겟이 항상 하나
		self.pr.grab_pin(pin) #asm_msg.child.pin

		robot = self.pr.hand_over_pin_check(pin, target) #asm_msg.parent.target.name
		self.pr.hold_assist(robot, "chair_part2", target)
		self.pr.fine_tune_insert_target(target, robot)
		# 학부 보조 연구원
		self.pr.insert_spiral_motion(robot)
		self.pr.hold_assist_reset(robot, "chair_part2", target)


	def insert_part(self, asm_msg):
		# rob1, rob2 작업, rob1이 작업 중심
		sorted_insert_target_poses = self.pr.sort_insert_target(asm_msg.parents)
		## move() - parent part는 고정, child part가 rob1이 작업을 할 수 있는 위치에 없으면 할수 있는 위치로 옮긴다
		is_moved = self.pr.hand_over_part(sorted_insert_target_poses, asm_msg)
		self.pr.grab_part(asm_msg.child, is_moved)
		self.pr.insert_part_motion(sorted_insert_target_poses[0])


	def insert_part_test(self, pa_name, pa_hole_list, ch_name, ch_hole_list):
		# rob1, rob2 작업, rob1이 작업 중심
		self.pr.send_tf(pa_name, pa_hole_list, ch_name, ch_hole_list) #중심으로 tf 재 설정

		robot = self.pr.hand_over_part_check(ch_name)
		# self.pr.grab_part(ch_name)

		# sorted_insert_target_poses = self.pr.sort_insert_target(asm_msg.parents)
		# ## move() - parent part는 고정, child part가 rob1이 작업을 할 수 있는 위치에 없으면 할수 있는 위치로 옮긴다
		# is_moved = self.pr.hand_over_part(sorted_insert_target_poses, asm_msg)
		# self.pr.grab_part(asm_msg.child, is_moved)
		# self.pr.insert_part_motion(sorted_insert_target_poses[0])



def main():

	a = Assembly_mode()

	print 'start?'
	raw_input()
	a.insert_part_test("chair_part6", ["hole6-3", "hole6-4"], "chair_part3", ["hole3-3", "hole3-4"])
	
	

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c122620_2", "hole2-6")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_1", "hole6-1")

	# print 'start?'
	# raw_input()
	# a.insert_pin_test("c101350_2", "hole6-4")
	



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass