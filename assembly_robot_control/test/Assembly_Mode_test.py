#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from Assembly_Process_test import Assembly_process
from assembly_robot_msgs.srv import asm_Srv, chan_Srv
from assembly_robot_msgs.msg import TransStamped
from std_srvs.srv  import SetBool


class Assembly_mode():

	def __init__(self):
		rospy.init_node('Assembly_Mode', anonymous=True)

		self.pr = Assembly_process(rospy)

		self.srv = rospy.Service('to_RobotControl', asm_Srv, self.Asm_callback)

		self.srv = rospy.Service('to_HoleCheck', asm_Srv, self.Asm_tfupdate_server)
		#chan cotrol
		self.chan = rospy.Service('chan_con', chan_Srv, self.chan_CB)

		rospy.wait_for_service('update_tf')
		rospy.wait_for_service('camera_server_1')
		rospy.wait_for_service('camera_server_2')
		
		tf_update = rospy.ServiceProxy('update_tf', SetBool)
		tf_update(True)
		
		# asm = TransStamped().TransStamped.header

		# print asm

		print "set"

	def chan_CB(self, srv):
		if srv.mode is 0: #camera mode
			self.pr.chan_camera_pose(srv.hole_name , srv.robot)
			return True, "camera_pose"
		elif srv.mode is 1: #mode mode
			self.pr.am.move_current_to(srv.x, srv.y, srv.z, srv.robot)
			return True, "move_current"
		elif srv.mode is 2: #mode insert
			self.pr.chan_insert_pose(srv.robot)
			return True, "insert_pose"

		else:
			return False, "no mode!"

	def Asm_tfupdate_server(self, data):

		print "Asm_tf_update_server"

		pin_list = []

		for i in data.parent.holepin:
			# print "name : {0}, holepin : {1}".format(data.parent.name, i)
			pin_list.append(self.pr.hole_find_update(data.parent.name[0], i))


		asm_pose = self.pr.hc_send_tf(data.parent.name[0], data.parent.holepin, data.child.name[0], data.child.holepin)

		asm_pose, pin_list = self.TransStamped(asm_pose, pin_list)

		print "asm_pose : {0},\n\n    pin_list : {1}".format(asm_pose, pin_list)

		return True, asm_pose, pin_list





		

	def Asm_callback(self, data):

		asm_pose = TransStamped().TransStamped # sendtf target
		# pin_pose = TransStamped() # update pin

		#pin_pose.TransStamped.transform.rotation.z = 3

		pin_list = []
		for i in range(len(data.parent.holepin)):
			temp_msg = TransStamped()
			temp_msg.TransStamped.transform.translation.z = 3

		# 실제 작업용 
		if data.type == 'insert':
			if 'c101350' in data.child.name[0] or 'c122620' in data.child.name[0]:
				print "pin"
				_result, pin_pose = self.insert_pin_test(data.child.name[0], data.parent.holepin[0], data.parent.name[0])
				pin_list.append(pin_pose)
				asm_pose = pin_pose
			else:

				# result, asm_pose, pin_list = self.insert_part_test(data.parent.name[0], data.parent.holepin, data.child.name[0], data.child.holepin)
				print "pa_name : {0}, pa_hole : {1}, ch_name : {2}, ch_hole : {3}".format(data.parent.name[0], data.parent.holepin, data.child.name[0], data.child.holepin)
				self.insert_part_test(data.parent.name[0], data.parent.holepin, data.child.name[0], data.child.holepin)
		

		#현철이 테스트용#
		#self.hc_test(self, data)


		asm_pose, pin_list = self.TransStamped(asm_pose, pin_list)

		# print "asm_pose : {0},\n\n    pin_list : {1}".format(asm_pose, pin_list)

		# return True, asm_pose, pin_list # _result, asm_pose, pin_list





	def insert_pin_test(self, pin, target, pa_part):
		################################################################################
		self.pr.grab_pin(pin) #asm_msg.child.pin

		robot = self.pr.hand_over_pin_check(pin, target) #asm_msg.parent.target.name
		# if pin not in ['c122620_1', 'c122620_2', 'c122620_3', 'c122620_4']: 
		self.pr.hold_assist(robot, pa_part, target)
		pin_pose = self.pr.fine_tune_insert_target(pa_part, target, robot)
		# 학부 보조 연구원
		self.pr.insert_spiral_pin_motion(robot)
		# if pin not in ['c122620_1', 'c122620_2', 'c122620_3', 'c122620_4']: 
		self.pr.hold_assist(robot, pa_part, target, reset=True)
		################################################################################
		# self.pr.insert_spiral_pin_motion(False)

		# print pin_pose

		return True, pin_pose


	def insert_part_test(self, pa_name, pa_hole_list, ch_name, ch_hole_list):
		# rob1, rob2 작업, rob1이 작업 중심
		self.pr.am.init_pose()
		print "pa_name : {0}, pa_hole : {1}, ch_name : {2}, ch_hole : {3}".format(pa_name, pa_hole_list, ch_name, ch_hole_list)
		
		# result, asm_pose, pin_list = self.pr.send_tf(pa_name, pa_hole_list, ch_name, ch_hole_list) #test 용

		robot, goal, pin_list = self.pr.hand_over_part_check(ch_name, ch_hole_list, pa_name, pa_hole_list)
		# self.pr.hold_assist(robot, pa_name, pa_hole_list[0])
		trans_ = self.pr.grab_part(robot, ch_name, pin_list, goal)
		_result, asm_pose = self.pr.insert_spiral_part_motion(robot, trans_)
		self.pr.hold_assist(robot, pa_name,  pa_hole_list[0], reset=True)

		return _result, asm_pose, pin_list


	def TransStamped(self, asm_pose, pin_list):


		asm = TransStamped()
		b = TransStamped()
		pin = []

		# print "asm_pose : {0} \n\n    pin_list : {1}".format(asm_pose, pin_list)

		asm.TransStamped.header.frame_id = asm_pose.header.frame_id
		asm.TransStamped.child_frame_id = asm_pose.child_frame_id
		asm.TransStamped.transform.translation.x = asm_pose.transform.translation.x
		asm.TransStamped.transform.translation.y = asm_pose.transform.translation.y
		asm.TransStamped.transform.translation.z = asm_pose.transform.translation.z
		asm.TransStamped.transform.rotation.x = asm_pose.transform.rotation.x
		asm.TransStamped.transform.rotation.y = asm_pose.transform.rotation.y
		asm.TransStamped.transform.rotation.z = asm_pose.transform.rotation.z
		asm.TransStamped.transform.rotation.w = asm_pose.transform.rotation.w

		for j in pin_list:

			b.TransStamped.header.frame_id = j.header.frame_id
			b.TransStamped.child_frame_id = j.child_frame_id
			b.TransStamped.transform.translation.x = j.transform.translation.x
			b.TransStamped.transform.translation.y = j.transform.translation.y
			b.TransStamped.transform.translation.z = j.transform.translation.z
			b.TransStamped.transform.rotation.x = j.transform.rotation.x
			b.TransStamped.transform.rotation.y = j.transform.rotation.y
			b.TransStamped.transform.rotation.z = j.transform.rotation.z
			b.TransStamped.transform.rotation.w = j.transform.rotation.w

			pin.append(b)



		return asm, pin



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