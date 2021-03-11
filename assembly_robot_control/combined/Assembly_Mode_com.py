#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from Assembly_Process_com import Assembly_process
from assembly_robot_msgs.srv import asm_Srv, chan_Srv
from assembly_robot_msgs.msg import TransStamped
from std_srvs.srv  import SetBool


class Assembly_mode():

	def __init__(self):
		rospy.init_node('Assembly_Mode', anonymous=True)

		

		self.pr = Assembly_process(rospy)

		self.srv = rospy.Service('to_RobotControl', asm_Srv, self.Asm_callback)

		#chan cotrol

		# rospy.wait_for_service('update_tf')
		# rospy.wait_for_service('camera_server_1')
		# rospy.wait_for_service('camera_server_2')
		
		# tf_update = rospy.ServiceProxy('update_tf', SetBool)
		# tf_update(True)

		
		# asm = TransStamped().TransStamped.header

		# print asm

		print "set"
		

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
			if 'C101350' in data.child.name[0] or 'C122620' in data.child.name[0]:
				print "pin"
				_result, pin_pose = self.insert_pin_test(data.child.name[0], data.parent.holepin[0], data.parent.name[0])
				pin_list.append(pin_pose)
				asm_pose = pin_pose
			else:

				# result, asm_pose, pin_list = self.insert_part_test(data.parent.name[0], data.parent.holepin, data.child.name[0], data.child.holepin)
				print "pa_name : {0}, pa_hole : {1}, ch_name : {2}, ch_hole : {3}".format(data.parent.name[0], data.parent.holepin, data.child.name[0], data.child.holepin)
				self.insert_part_test(data.parent.name, data.parent.holepin, data.child.name, data.child.holepin)
		
		elif data.type == 'screw':
			if 'C104322' in data.child.name[0]:
				print 'screw'
				_result, pin_pose = self.screw_pin_test(data.child.name[0], data.parent.holepin[0], data.parent.name[0])


		elif data.type == 'half_screw':
			pass
			
		elif data.type == 'total_screw':
			pass

		elif data.type == 'attach':
			pass

		elif data.type == 'rotate':
			pass
		#현철이 테스트용#
		#self.hc_test(self, data)


		asm_pose, pin_list = self.TransStamped(asm_pose, pin_list)

		# print "asm_pose : {0},\n\n    pin_list : {1}".format(asm_pose, pin_list)

		return True, asm_pose, pin_list # _result, asm_pose, pin_list


	def screw_pin_test(self, pin, part_hole, part_name):

		robot = self.pr.hand_over_hole_check(part_hole)

		# self.pr.grab_pin(robot, pin)

		# # self.pr.hold_assist(robot, part_name, part_hole)

		# pin_pose = self.pr.fine_tune_insert_target(part_name, part_hole, robot)

		# self.pr.insert_spiral_pin_motion(robot)

		pin_pose = self.pr.fine_tune_insert_target(part_name, part_hole, robot, move = False)

		self.pr.grab_screw_tool(robot, pin)


		self.pr.screw_drive_motion(robot, pin_pose)

		# self.pr.hold_assist(robot, part_name, part_hole, reset=True)



	def insert_pin_test(self, pin, part_hole, pa_part):
		################################################################################
		# robot = self.pr.hand_over_pin_check(pin, part_hole) #asm_msg.parent.target.name

		robot = self.pr.hand_over_hole_check(part_hole)
		
		self.pr.grab_pin(robot, pin) #asm_msg.child.pin

		# if pin not in ['c122620_1', 'c122620_2', 'c122620_3', 'c122620_4']: 
		self.pr.hold_assist(robot, pa_part, part_hole)
		pin_pose = self.pr.fine_tune_insert_target(pa_part, part_hole, robot)
		# 학부 보조 연구원
		self.pr.insert_spiral_pin_motion(robot)
		# if pin not in ['c122620_1', 'c122620_2', 'c122620_3', 'c122620_4']: 
		self.pr.hold_assist(robot, pa_part, part_hole, reset=True)
		################################################################################
		# self.pr.insert_spiral_pin_motion(False)

		# print pin_pose

		return True, pin_pose


	def insert_part_test(self, pa_name, pa_hole_list, ch_name, ch_hole_list):
		# rob1, rob2 작업, rob1이 작업 중심

		


		if len(pa_name) < 2:
			self.pr.am.init_pose()
			# print "pa_name : {0}, pa_hole : {1}, ch_name : {2}, ch_hole : {3}".format(pa_name, pa_hole_list, ch_name, ch_hole_list)
			
			# result, asm_pose, pin_list = self.pr.send_tf(pa_name, pa_hole_list, ch_name, ch_hole_list) #test 용
			group = self.pr.group_to_hole(ch_hole_list)

			# print "group : {0}".format(group)

			robot, goal, pin_list, sort_list = self.pr.hand_over_part_check(ch_name[0], group, pa_name[0], pa_hole_list)

			# self.pr.hold_assist(robot, pa_name, pa_hole_list[0])
			trans_ = self.pr.grab_part(robot, ch_name[0], pin_list, goal)
			_result, asm_pose = self.pr.insert_spiral_part_motion(robot, trans_, sort_list, ch_name[0])
			self.pr.hold_assist(robot, pa_name[0],  pa_hole_list[0], reset=True)

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


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass