#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from Assembly_Process_test import Assembly_process
from assembly_robot_msgs.srv import asm_Srv
from assembly_robot_msgs.msg import PoseStamped
from std_srvs.srv  import SetBool


class Assembly_mode():

	def __init__(self):
		rospy.init_node('Assembly_Mode', anonymous=True)

		self.pr = Assembly_process(rospy)

		self.srv = rospy.Service('to_RobotControl', asm_Srv, self.Asm_callback)

		# rospy.wait_for_service('update_tf')
		# rospy.wait_for_service('camera_server_1')
		# rospy.wait_for_service('camera_server_2')
		
		# tf_update = rospy.ServiceProxy('update_tf', SetBool)
		# tf_update(True)
		

		print "set"

	def Asm_callback(self, data):

		asm_pose = PoseStamped()
		pin_pose = PoseStamped()
		pin_list = []
		if data.type == 'insert':
			if 'c101350' in data.child.name[0] or 'c122620' in data.child.name[0]:
				print "pin"
				pin_pose = self.insert_pin_test(data.child.name[0], data.parent.holepin[0], data.parent.name[0])
				pin_list.append(pin_pose)
			else:
				asm_pose, pin_list = self.insert_part_test(data.parent.name[0], data.parent.holepin, data.child.name[0], data.child.holepin)
		
		return True, asm_pose, [pin_pose] # _result, asm_pose, pin_list

	# def insert_pin(self, asm_msg):
	# 	# (일단은) 모두 rob1 이 작업 
	# 	# real_insert_target_pose = self.process.fine_tune_insert_target(asm_msg.parent.target) # pin일 때는 parent 타겟이 항상 하나
	# 	self.pr.grab_pin(pin) #asm_msg.child.pin

	# 	robot = self.pr.hand_over_pin_check(pin, target) #asm_msg.parent.target.name
	# 	self.pr.hold_assist(robot, "chair_part2", target)
	# 	self.pr.fine_tune_insert_target(target, robot)
	# 	# 학부 보조 연구원
	# 	self.pr.insert_spiral_pin_motion(robot)
	# 	self.pr.hold_assist_reset(robot, "chair_part2", target)



	def insert_pin_test(self, pin, target, pa_part):
		# (일단은) 모두 rob1 이 작업 
		# real_insert_target_pose = self.process.fine_tune_insert_target(asm_msg.parent.target) # pin일 때는 parent 타겟이 항상 하나
		self.pr.grab_pin(pin) #asm_msg.child.pin

		robot = self.pr.hand_over_pin_check(pin, target) #asm_msg.parent.target.name
		# if pin not in ['c122620_1', 'c122620_2', 'c122620_3', 'c122620_4']: 
		self.pr.hold_assist(robot, pa_part, target)
		pin_pose = self.pr.fine_tune_insert_target(pa_part, target, robot)
		# 학부 보조 연구원
		self.pr.insert_spiral_pin_motion(robot)
		# if pin not in ['c122620_1', 'c122620_2', 'c122620_3', 'c122620_4']: 
		self.pr.hold_assist(robot, pa_part, target, reset=True)

		return True, pin_pose


	# def insert_part(self, asm_msg):
	# 	# rob1, rob2 작업, rob1이 작업 중심
	# 	sorted_insert_target_poses = self.pr.sort_insert_target(asm_msg.parents)
	# 	## move() - parent part는 고정, child part가 rob1이 작업을 할 수 있는 위치에 없으면 할수 있는 위치로 옮긴다
	# 	is_moved = self.pr.hand_over_part(sorted_insert_target_poses, asm_msg)
	# 	self.pr.grab_part(asm_msg.child, is_moved)
	# 	self.pr.insert_part_motion(sorted_insert_target_poses[0])


	def insert_part_test(self, pa_name, pa_hole_list, ch_name, ch_hole_list):
		# rob1, rob2 작업, rob1이 작업 중심
		self.pr.am.init_pose()

		self.pr.send_tf(pa_name, pa_hole_list, ch_name, ch_hole_list) #중심으로 tf 재 설정

		robot = self.pr.hand_over_part_check(ch_name, ch_hole_list, pa_name, pa_hole_list)
		# self.pr.hold_assist(robot, pa_name, pa_hole_list[0])
		trans_ = self.pr.grab_part(robot, ch_name)
		self.pr.insert_spiral_part_motion(robot, trans_)
		self.pr.hold_assist(robot, pa_name,  pa_hole_list[0], reset=True)
		# sorted_insert_target_poses = self.pr.sort_insert_target(asm_msg.parents)
		## move() - parent part는 고정, child part가 rob1이 작업을 할 수 있는 위치에 없으면 할수 있는 위치로 옮긴다
		# is_moved = self.pr.hand_over_part(sorted_insert_target_poses, asm_msg)
		# self.pr.grab_part(asm_msg.child, is_moved)
		# self.pr.insert_part_motion(sorted_insert_target_poses[0])



def main():

	a = Assembly_mode()

	rospy.spin()
	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c122620_1", "hole2-5", "chair_part2")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c122620_2", "hole2-6", "chair_part2")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c122620_3", "hole3-5", "chair_part3")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c122620_4", "hole3-6", "chair_part3")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_1", "hole6-3", "chair_part6")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_1", "hole6-4", "chair_part6")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_1", "hole6-2", "chair_part6")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_1", "hole6-1", "chair_part6")
	

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_10", "hole6-5", "chair_part6")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_11", "hole6-6", "chair_part6")

	# print 'start? hole6-7'
	# raw_input()
	
	# a.insert_pin_test("c101350_12", "hole6-7", "chair_part6")

	# print 'start?'
	# raw_input()
	# a.insert_part_test("chair_part6", ["hole6-3", "hole6-4"], "chair_part3", ["hole3-3", "hole3-4"])

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_1", "hole3-1", "chair_part3")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_1", "hole3-2", "chair_part3")
	
	# print 'start?'
	# raw_input()
	# a.insert_part_test("chair_part6", ["hole6-1", "hole6-2"], "chair_part2", ["hole2-1", "hole2-2"])

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_1", "hole2-3", "chair_part2")

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_1", "hole2-4", "chair_part2")

	# print 'start?'
	# raw_input()
	# a.insert_part_test("chair_part6", ["hole6-5", "hole6-6", 'hole6-7'], "chair_part4", ["hole4-1", "hole4-2", 'hole4-6'])
	

	# print 'start?'
	# raw_input()
	
	# a.insert_pin_test("c101350_2", "hole6-1")

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