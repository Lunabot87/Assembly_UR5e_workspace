#!/usr/bin/env python
#-*- coding:utf-8 -*-

from std_msgs.msg import * 
from Assembly_Motion_test import Assembly_motion
from tf2_ros import *
import tf
from geometry_msgs.msg import *

import numpy as np
#import rospy
from assembly_robot_msgs.srv import *
import time
import copy

from assembly_robot_msgs.srv import *
from assembly_robot_msgs.msg import *

from utils.holepin import *
from utils.conversions import *

_KHOLECHECKOFFSET = 0.28 # 애들이 정함
#python 규칙 상수는 _대문자


class Assembly_process():

	def __init__(self, ros):

		self.rospy = ros
		self.am = Assembly_motion(ros)
		self.tfBuffer = Buffer()
		self.listener = TransformListener(self.tfBuffer)
		self.br = StaticTransformBroadcaster()

		self.mid_check = ros.ServiceProxy('check_ASM_possible', check_Srv)

		time.sleep(2)


	def elp_camera_client(self, name, robot):
		rob_cam = ''
		if robot is False:
			rob_cam = 'camera_server_1'
		else:
			rob_cam = 'camera_server_2'

		for count in range(10):

			# self.rospy.wait_for_service(rob_cam)
			try:
				rob_client = self.rospy.ServiceProxy(rob_cam, cam_Srv)
				basler_data = rob_client(name)
				# print basler_client
				return basler_data
				break
			except self.rospy.ServiceException as e:
				print("Service call failed: %s"%e)

		return False


	def hole_find_update(self, pa_part, hole_name):

		not_hand_over = ['C122620_1','C122620_2' ,'C122620_3' ,'C122620_4']

		rob1_trans = self.tfBuffer.lookup_transform('rob1_real_base_link', hole_name, self.rospy.Time(0))

		rob1_trans_l = self.list_from_trans(rob1_trans, euler=True)

		rob2_trans = self.tfBuffer.lookup_transform('rob2_real_base_link', hole_name, self.rospy.Time(0))

		rob2_trans_l = self.list_from_trans(rob2_trans, euler=True)

		dist = np.linalg.norm(np.array(rob1_trans_l[:3]))-np.linalg.norm(np.array(rob2_trans_l[:3]))
		if dist > 0:
			if pa_part in not_hand_over: #수정 필요 
				robot = False

			else:
				robot = True
		else:
			robot = False


		if robot is False:
			rob_camera = 'camera_center_1'
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
			ro_trans = [0, 0, 0, 0, 0, 1, 0.0000463]

		else:
			rob_camera = 'camera_center_2'
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'
			ro_trans = [0, 0, 0, 0, 0, 0, 1]

		camera_trans = [0.000, -0.072, 0.058,-0.707, 0.000, 0.000, 0.707]
		z_trans = [0,0,0.22,0,0,0,1]

		hole_trans = self.tfBuffer.lookup_transform('world', hole_name, self.rospy.Time(0))

		hole_trans = self.list_from_trans(hole_trans)

		# temp_t, temp_r = self.am.trans_convert(hole_trans[:3] + [0,0,0,1], ro_trans)

		# temp_ = temp_t.tolist() + temp_r.tolist()

		temp_ = hole_trans[:3] + ro_trans[3:]

		temp_t= self.am.trans_convert(temp_, z_trans)

		# trans_g = self.tfBuffer.lookup_transform(rob_camera, ee_link, self.rospy.Time(0))

		temp_t= self.am.trans_convert(temp_t, camera_trans)

		base_ = self.tfBuffer.lookup_transform(base_link, 'world', self.rospy.Time(0))

		base_t = self.am.trans_convert(self.list_from_trans(base_), temp_t)

		base_t = self.rot_arrange(base_t) ##문제시 이거 삭제 

		# result = self.am.move_motion(base_t[:3], base_t[3:], 0, robot, c=True) #c = 카메라 화면처럼 움직여야 할때

		# if result < 0: return -1 #error 상태 수정 
			
		#########카메라사용########
		time.sleep(2)

		#trans_ = self.client(pa_part, hole_name, robot)
		trans_ = False

		if trans_ is not False: 
			# self.br.sendTransform(self.trans_from_list(trans_, 'world', 'target_g0'))
			hole_trans = self.tfBuffer.lookup_transform(base_link, 'world', self.rospy.Time(0))
			hole_trans = self.am.trans_convert(self.list_from_trans(hole_trans), trans_)

			return_hole = self.tfBuffer.lookup_transform(pa_part, 'world', self.rospy.Time(0))
			return_hole = self.am.trans_convert(self.list_from_trans(return_hole), trans_)
			return_hole = self.trans_from_list(return_hole, pa_part, hole_name)

		else:
			# print "name : {0}, holepin : {1}".format(pa_part, hole_name)
			return_hole = self.tfBuffer.lookup_transform(pa_part, hole_name, self.rospy.Time(0))

		return return_hole






	def fine_tune_insert_target(self, pa_part, hole_name, robot, move = True):
		# target_pose[PoseStamped] : 핀을 꽂은 상태에서 eef의 목표 값
		# kHoleCheckOffset
		# 7/22 적용
		# xyz, rpy = self.listener.lookupTransform('/world', part_name, self.rospy.Time(0))

		# self.am.camera_pose(robot)

		if robot is False:
			rob_camera = 'camera_center_1'
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
			ro_trans = [0, 0, 0, 0, 0, 1, 0.0000463]
			# ro_trans = [0, 0, 0, 0, 0, 0, 1]

		else:
			rob_camera = 'camera_center_2'
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'
			ro_trans = [0, 0, 0, 0, 0, 0, 1]

		camera_trans = [0.000, -0.072, 0.058,-0.707, 0.000, 0.000, 0.707]
		z_trans = [0,0,0.22,0,0,0,1]

		hole_trans = self.tfBuffer.lookup_transform('world', hole_name, self.rospy.Time(0))

		hole_trans = self.list_from_trans(hole_trans)

		# temp_t, temp_r = self.am.trans_convert(hole_trans[:3] + [0,0,0,1], ro_trans)

		# temp_ = temp_t.tolist() + temp_r.tolist()

		temp_ = hole_trans[:3] + ro_trans[3:]

		temp_t= self.am.trans_convert(temp_, z_trans)

		# trans_g = self.tfBuffer.lookup_transform(rob_camera, ee_link, self.rospy.Time(0))

		temp_t= self.am.trans_convert(temp_t, camera_trans)

		base_ = self.tfBuffer.lookup_transform(base_link, 'world', self.rospy.Time(0))

		base_t = self.am.trans_convert(self.list_from_trans(base_), temp_t)

		base_t = self.rot_arrange(base_t) ##문제시 이거 삭제 

		result = self.am.move_motion(base_t[:3], base_t[3:], 0, robot, c=True) #c = 카메라 화면처럼 움직여야 할때

		if result < 0: return -1 #error 상태 수정 
			
		#########카메라사용########
		time.sleep(2)

		trans_ = self.client(pa_part, hole_name, robot)
		# trans_ = False

		#핀 커넥시 사용하는 값

		if move is True:
			if trans_ is not False: 
				# self.br.sendTransform(self.trans_from_list(trans_, 'world', 'target_g0'))
				hole_trans = self.tfBuffer.lookup_transform(base_link, 'world', self.rospy.Time(0))
				hole_trans = self.am.trans_convert(self.list_from_trans(hole_trans), trans_)

				return_hole = self.tfBuffer.lookup_transform(pa_part, 'world', self.rospy.Time(0))
				return_hole = self.am.trans_convert(self.list_from_trans(return_hole), trans_)
				return_hole = self.trans_from_list(return_hole, pa_part, hole_name)

				hole_trans = self.trans_from_list(hole_trans, '', '')
				hole_trans_l = self.list_from_trans(hole_trans, euler=True)

			else:
				return_hole = self.tfBuffer.lookup_transform(pa_part, hole_name, self.rospy.Time(0))
				hole_trans = self.tfBuffer.lookup_transform(base_link, hole_name, self.rospy.Time(0))

				hole_trans_l = self.list_from_trans(hole_trans, euler=True)


			asm_pose = TransStamped()
			pin_pose = TransStamped()
			pin_list = []

			asm_pose.TransStamped = return_hole
			pin_pose.TransStamped = return_hole
			pin_list.append(pin_pose)

			# result = self.mid_check(asm_pose, pin_list)
			# if result is False: return -1

			hole_trans_l[2] += 0.18 #why use?

			if hole_name not in ['hole2-5', 'hole2-6', 'hole3-5', 'hole3-6']:
				hole_trans_l[3] = 3.1415
				hole_trans_l[4] = 0
				hole_trans_l[5] = -1.5707

		# xyz.append(3.1415)
		# xyz.append(0)
		# xyz.append(0)
		# self.am.move_to(xyz, robot)

		
			result = self.am.move_motion(hole_trans_l[:3], tf.transformations.quaternion_from_euler(hole_trans_l[3],hole_trans_l[4],hole_trans_l[5]), 0.10, robot, collision=False)
			if result < -1: return -1 

			# print return_hole

			return  hole_trans, self.tfBuffer.lookup_transform(pa_part, 'update', self.rospy.Time(0))#return_hole#hole_trans #transposeStamped

		else:
			if trans_ is not False: 
				return trans_ , self.tfBuffer.lookup_transform(pa_part, 'update', self.rospy.Time(0))

			else:

				hole_trans = self.tfBuffer.lookup_transform('world', hole_name, self.rospy.Time(0))

				return self.list_from_trans(hole_trans), self.tfBuffer.lookup_transform(pa_part, 'update', self.rospy.Time(0))


		return False



	def group_to_hole(self, ch_hole_name):

		hole_list = []

		for name in ch_hole_name:
			hole_list.append(self.list_from_trans(self.tfBuffer.lookup_transform('world', name, self.rospy.Time(0)))) #list

		count = len(hole_list)
		group_ = [[ch_hole_name[0]]]

		

		for i in range(count):
			temp_list = []
			temp_list.append(ch_hole_name[i])
			for j in range(count-(1+i)):

				dist = np.linalg.norm(np.array(hole_list[i][:2]))-np.linalg.norm(np.array(hole_list[i+j+1][:2]))
				# print "dist : {0}".format(dist)
				if abs(dist) < 0.05:
					temp_list.append(ch_hole_name[i+j+1])
					# print "temp_list : {0}".format(temp_list)

			

			if len(temp_list) <= count:
				for num in range(len(group_)):
					if temp_list[0] in group_[num]:
						for k in range(len(temp_list)-1):
							group_[num].append(temp_list[k+1])
						break
					else:
						group_.append(temp_list)
						break


			else:
				for num in range(len(group_)):
					if temp_list in group_[num]:
						continue
					else:
						group_.append(temp_list)
						break


		return group_






	######################################## chan_control ########################################

	def chan_camera_pose(self, hole_name, robot):
		# target_pose[PoseStamped] : 핀을 꽂은 상태에서 eef의 목표 값
		# kHoleCheckOffset
		# 7/22 적용
		# xyz, rpy = self.listener.lookupTransform('/world', part_name, self.rospy.Time(0))

		# self.am.camera_pose(robot)

		if robot is False:
			rob_camera = 'camera_center_1'
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
			ro_trans = [0, 0, 0, 0, 0, 1, 0.0000463]

		else:
			rob_camera = 'camera_center_2'
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'
			ro_trans = [0, 0, 0, 0, 0, 0, 1]

		camera_trans = [0.000, -0.072, 0.058,-0.707, 0.000, 0.000, 0.707]
		z_trans = [0,0,0.22,0,0,0,1]

		hole_trans = self.tfBuffer.lookup_transform('world', hole_name, self.rospy.Time(0))

		hole_trans = self.list_from_trans(hole_trans)

		# temp_t, temp_r = self.am.trans_convert(hole_trans[:3] + [0,0,0,1], ro_trans)

		# temp_ = temp_t.tolist() + temp_r.tolist()

		temp_ = hole_trans[:3] + ro_trans[3:]

		temp_t= self.am.trans_convert(temp_, z_trans)

		# trans_g = self.tfBuffer.lookup_transform(rob_camera, ee_link, self.rospy.Time(0))

		temp_t= self.am.trans_convert(temp_t, camera_trans)

		base_ = self.tfBuffer.lookup_transform(base_link, 'world', self.rospy.Time(0))

		base_t = self.am.trans_convert(self.list_from_trans(base_), temp_t)

		base_t = self.rot_arrange(base_t) ##문제시 이거 삭제 

		result = self.am.move_motion(base_t[:3], base_t[3:], 0, robot, c=True) #c = 카메라 화면처럼 움직여야 할때

		if result < 0: return -1 #error 상태 수정 
			


	def chan_insert_pose(self, robot):

		if robot is False:
			rob_camera = 'camera_center_1'
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
			ro_trans = [0, 0, 0, 0, 0, 1, 0.0000463]

		else:
			rob_camera = 'camera_center_2'
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'
			ro_trans = [0, 0, 0, 0, 0, 0, 1]

		
		hole_trans = self.tfBuffer.lookup_transform(base_link, rob_camera, self.rospy.Time(0))

		hole_trans_l = self.list_from_trans(hole_trans, euler=True)


		# hole_trans_l[2] = 0.30 #why use?

		hole_trans_l[3] = 3.1415
		hole_trans_l[4] = 0
		hole_trans_l[5] = -1.5707


		result = self.am.move_motion(hole_trans_l[:3], tf.transformations.quaternion_from_euler(hole_trans_l[3],hole_trans_l[4],hole_trans_l[5]), 0.05, robot, collision=False)
		

		if result < -1: return 


	###############################################################################################







	def client(self, part, hole_name, robot):

		hole_ = self.tfBuffer.lookup_transform('world', hole_name, self.rospy.Time(0))

		hole_ = self.list_from_trans(hole_)

		trans_ = [0,0,0,0,0,0,0]
		
		for count in range(10):

			result = self.elp_camera_client(hole_name, robot)

			if result is False:
				print "not found"
				break
				return False
				

			else:
				
				if result.result is False:
					print "result : {0}".format(result)
					self.am.move_current_to(result.x, result.y, result.z, robot)

					trans_[0] = hole_[0] + result.x
					trans_[1] = hole_[1] + result.y

					self.br.sendTransform(self.trans_from_list(trans_[:2]+hole_[2:], 'world', 'update'))

					time.sleep(0.5)

				else:
					time.sleep(0.2)
					
					trans_ = self.tfBuffer.lookup_transform('world', result.name, self.rospy.Time(0))

					trans_ = self.list_from_trans(trans_)

					trans_[0] += result.x
					trans_[1] += result.y

					self.br.sendTransform(self.trans_from_list(trans_[:2]+hole_[2:], 'world', 'update'))

					return trans_[:2]+hole_[2:]
						
		return trans_[:2]+hole_[2:]



	def hand_over_pin_check(self, pin, target_name):

		# not_hand_over = []
		# not_hand_over = ['C122620_1', 'C122620_2', 'C122620_3', 'C122620_4',]

		rob1_trans = self.tfBuffer.lookup_transform('rob1_real_base_link', target_name, self.rospy.Time(0))

		rob1_trans_l = self.list_from_trans(rob1_trans, euler=True)

		rob2_trans = self.tfBuffer.lookup_transform('rob2_real_base_link', target_name, self.rospy.Time(0))

		rob2_trans_l = self.list_from_trans(rob2_trans, euler=True)

		dist = np.linalg.norm(np.array(rob1_trans_l[:3]))-np.linalg.norm(np.array(rob2_trans_l[:3]))
		if dist > 0:
			return False


			return True
		else:
			return False



	def hand_over_part_check(self, ch_name, group, pa_name, pa_hole_list):

		sort_list = []
		sort_hole_list = []
		sort_count = len(group)

		# print "group : {0}".format(group)

		for i in range(sort_count):
			trans = self.list_from_trans(self.tfBuffer.lookup_transform(ch_name+'-GRASP-3', group[i][0], self.rospy.Time(0)))
			print trans[:2]
			sort_list.append(np.linalg.norm(np.array(trans[:2]))) #list

		temp_sort = copy.deepcopy(sort_list)
		temp_sort.sort()

		for i in range(sort_count):
			sort_hole_list.append(group[sort_list.index(temp_sort[i])])
				
		robot = False

		# print "ch_hole_list : {0}".format(sort_hole_list)


		########

		test_ = self.tfBuffer.lookup_transform('world', pa_hole_list[0], self.rospy.Time(0))

		test_ = self.list_from_trans(test_)

		eef_mat = tf_to_mat(test_[:3], test_[3:])

		rot_zaxis = eef_mat[:3,2]

		z_axis = [0,0,1]

		########

		_goal, _target = self.send_tf(pa_name, pa_hole_list[:len(sort_hole_list[0])], ch_name, sort_hole_list[0])


		hold_ = self.tfBuffer.lookup_transform('rob1_real_base_link', ch_name+'-GRASP-3', self.rospy.Time(0))

		hold_ = self.list_from_trans(hold_)

		success = self.select_part_robot(robot, hold_, idx=True)

		if success is False:
			robot = True

			hold_ = self.tfBuffer.lookup_transform('rob2_real_base_link', ch_name+'-GRASP-3', self.rospy.Time(0))

			hold_ = self.list_from_trans(hold_)

			success = self.select_part_robot(robot, hold_, idx=True)
			#if success is not True: return -1


		tmp = self.tfBuffer.lookup_transform(ch_name, ch_name+'-GRASP-3', self.rospy.Time(0))

		tmp = self.list_from_trans(tmp)

		trans = self.am.trans_convert(_target, tmp)

		# trans_l = self.list_from_trans(trans)

		# temp_t = self.am.trans_convert(trans_l, [0,0,-0.155,0,0,0,1])

		temp_t = self.am.trans_convert(trans, [0,0,-0.165,0,0,0,1])

		trans_g = self.am.trans_convert(self.list_from_trans(self.tfBuffer.lookup_transform('world', pa_name, self.rospy.Time(0))), _goal) #수정중  

		# trans_g = self.rot_arrange(self.list_from_trans(trans_g))

		if (np.dot(z_axis, rot_zaxis)) > 0.8:
			print"-------"*100
			trans_g = self.hc_send_tf(pa_name, pa_hole_list[:len(sort_hole_list[0])], ch_name, sort_hole_list[0])


		temp_tg = self.am.trans_convert(trans_g, [0,0,0.05,0.9999997, 0, 0, 0.0007963])

		trans_ = self.am.trans_convert(temp_tg, temp_t)

		# trans = temp_t + temp_r


		self.br.sendTransform(self.trans_from_list(trans_, 'world', "real_goal")) #살리냐 죽이냐 그것이 문제로다 

		time.sleep(1)

		# print "real :   "
		# print self.tfBuffer.lookup_transform('real_goal', 'goal', self.rospy.Time(0))

		# success = self.select_part_robot(robot, "real_goal")

		success = self.select_part_robot(robot, trans_)

		if success is False and robot is False:
			robot = True
			success = self.select_part_robot(robot, trans_)
			#if success is not True:
			#	return -1

		hold_check = None

		pa_goal_list = []

		hole_list = []

		#########03/06 업데이트####

		self.hold_assist(robot, pa_name, pa_hole_list[0])

		for i, j in zip(pa_hole_list, range(len(pa_hole_list))):

			select = self.hand_over_hole_check(i)	

			trans_, null = self.fine_tune_insert_target(pa_name, i, robot, move = False)

			if trans_ is not False:

				self.br.sendTransform(self.trans_from_list(trans_, 'world', 'target_g'+str(j)))

				time.sleep(0.05)
				
			else:
				trans_ = self.tfBuffer.lookup_transform('world', i, self.rospy.Time(0))

				time.sleep(0.05)

				self.br.sendTransform(self.trans_from_list(self.list_from_trans(trans_), 'world', 'target_g'+str(j)))

				time.sleep(0.05)

			pa_goal_list.append('target_g'+str(j))

			hole_list.append(trans_)

			# if trans_ is False:
			# 	trans_ = self.tfBuffer.lookup_transform('world', i, self.rospy.Time(0))


		_goal, _target = self.send_tf(pa_name, pa_goal_list[:len(sort_hole_list[0])], ch_name, sort_hole_list[0])

		trans_g = self.am.trans_convert(self.list_from_trans(self.tfBuffer.lookup_transform('world', pa_name, self.rospy.Time(0))), _goal) #수정중  

		# trans_g = self.rot_arrange(self.list_from_trans(trans_g))

		temp_tg = self.am.trans_convert(trans_g, [0,0,-0.05,0.9999997, 0, 0, 0.0007963])

		if (np.dot(z_axis, rot_zaxis)) > 0.8:
			trans_g = self.hc_send_tf(pa_name, pa_hole_list[:len(sort_hole_list[0])], ch_name, sort_hole_list[0])

		trans = self.am.trans_convert(temp_tg, temp_t)

		# trans = temp_t + temp_r

		for i in range(5):
			self.br.sendTransform(self.trans_from_list(trans, 'world', "real_goal"))

			time.sleep(0.2)

		# print self.tfBuffer.lookup_transform('real_goal', 'goal', self.rospy.Time(0))
		
		# print "robot : {0}, trans : {1}, hole_list : {2}".format(robot, trans, hole_list)
		# print "type robot : {0}, trans : {1}, hole_list : {2}".format(type(robot), type(trans), type(hole_list))
		return robot, trans, hole_list, sort_hole_list #hole_list : list로 생성 되어있음 


	def hand_over_hole_check(self, target_name):

		rob1_trans = self.tfBuffer.lookup_transform('rob1_real_base_link', target_name, self.rospy.Time(0))

		rob1_trans_l = self.list_from_trans(rob1_trans, euler=True)

		rob2_trans = self.tfBuffer.lookup_transform('rob2_real_base_link', target_name, self.rospy.Time(0))

		rob2_trans_l = self.list_from_trans(rob2_trans, euler=True)

		dist = np.linalg.norm(np.array(rob1_trans_l[:3]))-np.linalg.norm(np.array(rob2_trans_l[:3]))
		
		if dist > 0:
			return True
		else:
			return False



	def select_part_robot(self, robot, goal, idx = False):

		if robot is False:
			base_frame = 'rob1_real_base_link'
		else:
			base_frame = 'rob2_real_base_link'


		if idx is not True: 
			# trans = self.tfBuffer.lookup_transform(base_frame, goal, self.rospy.Time(0))
			trans = self.tfBuffer.lookup_transform(base_frame, 'world', self.rospy.Time(0)) 
			trans = self.list_from_trans(trans)

			trans = self.am.trans_convert(trans, goal)

			result = self.am.move_motion(trans[:3], trans[3:], 0, robot, c=True, _check=True)
		
		else: 
			trans = goal
			trans[2] += 0.165


			result = self.am.move_motion(trans[:3], trans[3:], 0.1, robot, c=True, _check=True)

		return result

	def grab_pin(self, robot, pin_name):#asm_child_msg, is_moved):
		# grasp = self.make_grasp_msg(asm_child_msg.pin, asm_child_msg.pose)
		# self.motion.pick_up_pin(grasp)
		self.am.pick_up_pin(robot, pin_name)

	################################################현철이 코드 적용 부분###############################################
	def hc_send_tf(self, pa_name, pa_hole_list, ch_name, ch_hole_list):

		# print "pa name : {0}, holepin : {1}, ch name : {2}, holepin : {3}".format(pa_name, pa_hole_list, ch_name, ch_hole_list)

		goal_list = []
		target_list = []

		goal = [0,0,0,0,0,0]
		target = [0,0,0,0,0,0]

		for i, j in zip(pa_hole_list, ch_hole_list):
			goal_list.append(self.list_from_trans(self.tfBuffer.lookup_transform(pa_name, i, self.rospy.Time(0))))
			target_list.append(self.list_from_trans(self.tfBuffer.lookup_transform(ch_name, j, self.rospy.Time(0))))

		

		trans_, rot_, axis_ = get_asm_pose_by_HolePin(goal_list, target_list)

		# print "trans_ : {0}".format(trans_)

		# print "rot_ : {0}".format(rot_)

		self.br.sendTransform(self.trans_from_list(trans_.tolist()+rot_.tolist() , pa_name, "goal"))

		time.sleep(1)

		asm_pose = trans_.tolist()+rot_.tolist()
		#asm_pose[2] = 0.3

		pin_pose = []
		pin_pose.append([self.trans_from_list(pin , pa_name, pa_hole) for pin, pa_hole in zip(goal_list, pa_hole_list)])

		# print "asm_pose : {0}".format(asm_pose)

		self.br.sendTransform(self.trans_from_list(asm_pose, pa_name, "update_goal"))

		return asm_pose# self.trans_from_list(asm_pose, pa_name, ch_name)


	###########################################11/05###########################################	

	def send_tf(self, pa_name, pa_hole_list, ch_name, ch_hole_list):

		goal_list = []
		target_list = []

		goal = [0,0,0,0,0,0]
		target = [0,0,0,0,0,0]


		for i, j in zip(pa_hole_list, ch_hole_list):
			goal_list.append(self.tfBuffer.lookup_transform(pa_name, i, self.rospy.Time(0)))
			target_list.append(self.tfBuffer.lookup_transform(j, ch_name, self.rospy.Time(0)))



		
		for i, j in zip(goal_list, target_list):
			temp_g = self.list_from_trans(i, euler=True)
			temp_t = self.list_from_trans(j, euler=True)
			for k in range(6):
				if k < 3:
					goal[k] += temp_g[k]
					target[k] += temp_t[k]
				else:
					goal[k] = temp_g[k]
					target[k] = temp_t[k]

		for i in range(3):
			goal[i] = goal[i]/len(pa_hole_list)
			target[i] = target[i]/len(ch_hole_list)

		
		# print goal[4]

		if goal[4] < 0:
			if abs(goal[4] + 3.14) < 0.1:
				goal[4] = -3.1415
			elif abs(goal[4] + 1.5707) < 0.1:
				goal[4] = -1.5707
			else:
				goal[4] = 0
		else:
			if abs(goal[4] - 3.14) < 0.1:
				goal[4] = 3.1415
			elif abs(goal[4] - 1.5707) < 0.1:
				goal[4] = 1.5707
			else:
				goal[4] = 0

		# print goal[4]


		self.br.sendTransform(self.trans_from_list(goal, pa_name, "goal", euler=True))
		self.br.sendTransform(self.trans_from_list(target, ch_name, "target", euler=True))

		# time.sleep(1)

		goal = self.list_from_trans(self.trans_from_list(goal, pa_name, "goal", euler=True))
		target = self.list_from_trans(self.trans_from_list(target, ch_name, "target", euler=True))

		print "goal  :  {0},  target  :  {1}".format(goal, target)

		return goal, target


	def grab_part(self, robot, ch_name, pin_list, goal):

		Path = "/home/care/ur_ws/src/Assembly_UR5e_workspace/assembly_robot_control/ulol_tf/object_description/chair_meshes/"

		if robot is False:
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
		else:
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'

		trans = self.tfBuffer.lookup_transform(base_link, ch_name+'-GRASP-3', self.rospy.Time(0))

		trans = self.list_from_trans(trans)
		
		trans[2] += 0.180 #why use?

		self.am.hold_assistant(trans[:3], trans[3:], 0.1, robot)

		trans_ = self.tfBuffer.lookup_transform(ee_link, ch_name, self.rospy.Time(0))

		self.am.attach(robot, ch_name, Path + 'chair part' + ch_name[-1] + ".SLDPRT.STL")

		time.sleep(0.5)

		# trans_ = self.list_from_trans(trans_)

		z_trans = self.tfBuffer.lookup_transform('real_goal', ee_link,  self.rospy.Time(0))

		trans = self.tfBuffer.lookup_transform('world', ee_link, self.rospy.Time(0))

		trans = self.list_from_trans(trans)

		trans[2] += (z_trans.transform.translation.z*0.7)

		print "trans[2] : {0}".format(trans)
		raw_input()

		way = self.pose_from_list(trans, '' ,'')

		self.am.cartestian_move(robot, way.pose)

		#self.am.move_motion(trans[:3], trans[3:], 0.1, robot)

		# trans = self.tfBuffer.lookup_transform(base_link, 'real_goal', self.rospy.Time(0))

		trans = self.tfBuffer.lookup_transform('world', 'real_goal', self.rospy.Time(0))

		trans = self.list_from_trans(trans)

		# trans = self.am.trans_convert(trans, goal)

		# self.am.move_motion(trans[:3], trans[3:], 0, robot, c=True)



		pose = self.pose_from_list(trans, '', '')

		waypoint = pose.pose

		# print waylist

		self.am.cartestian_move(robot, waypoint)


		return trans_


	def grab_screw_tool(self,robot, tool):
		if robot is False:
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
		else:
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'

		self.am.grab_tool(robot, tool)


	def screw_drive_motion(self, robot, goal):
		if robot is False:
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
			rob = "rob1_"
		else:
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'
			rob = "rob2_"

		print "screw_drive"
		raw_input()

		trans = self.tfBuffer.lookup_transform(rob + 'screw_tool_link', ee_link, self.rospy.Time(0))

		# trans_ = self.am.trans_convert(self.list_from_trans(goal), [0,0,-0.15,0,0,0,0])

		# trans_ = self.list_from_trans(goal)

		# world = self.tfBuffer.lookup_transform(base_link, 'world', self.rospy.Time(0))

		trans_ = goal

		trans_[2] += 0.1

		# trans_ = self.am.trans_convert(self.list_from_trans(world), trans_)

		trans = self.am.trans_convert(trans_, self.list_from_trans(trans))

		pose = self.pose_from_list(trans, '', '')

		waypoint = pose.pose

		# print waylist

		self.am.cartestian_move(robot, waypoint)

		# self.am.move_motion(trans[:3], trans[3:], 0, robot)

		start = self.am.current_pose(robot)	

		self.am.screw_motion(robot)


		
	def insert_spiral_pin_motion(self, robot, num_of_trial=5):
		# spiral() 실행, 성공할 때까지 num_of_trial 만큼 반복
		# target pose와 grasp_config.yaml 의 데이터를 합쳐서 approach, retreat도 결정 
		# for i in range(num_of_trial):
		# 	if self.am.sprial_motion():
		# 		break
		start = self.am.current_pose(robot)	
		for count in range(num_of_trial):
			result = self.am.sprial_pin(robot, gripper = 255)
			if result is not True:
				self.am.move_current_up(0.1, robot)
				self.am.current_pose(robot, reset =True, pose = start)
				x = 0 if count > 2 else (1 - count*2)
				y = 0 if count < 2 else (count-3)

				self.am.move_current_to(x*0.01, y*0.01, 0,robot)
			else:
				break
		self.am.gripper_control(robot, 0)

		self.am.init_pose(robot)
				
	def insert_spiral_part_motion(self, robot, trans_, sort_list, ch_name, num_of_trial=5):
		# spiral() 실행, 성공할 때까지 num_of_trial 만큼 반복
		# target pose와 grasp_config.yaml 의 데이터를 합쳐서 approach, retreat도 결정 
		# for i in range(num_of_trial):
		# 	if self.am.sprial_motion():
		# 		break


		if robot is False:
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
		else:
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'


		
		result = self.am.sprial_pin(robot, gripper = 255)#, 1)

		self.am.dettach(robot, trans_.child_frame_id)

		trans = self.tfBuffer.lookup_transform('world', trans_.header.frame_id, self.rospy.Time(0))

		trans = self.am.trans_convert(self.list_from_trans(trans), self.list_from_trans(trans_))

		print "sort_list : {0}".format(sort_list)

		trans_ = self.trans_from_list(trans, 'world', trans_.child_frame_id)

		self.br.sendTransform(trans_)


		if len(sort_list) < 2:

			self.am.gripper_control(robot, 0)

			

		else:	
			self.am.torque_mode(robot, [0,0,1,0,1,1], [0,0,-50,0,0,0]) #torque z y yaw 풀고
			temp_base = 'rob2_real_base_link' if robot is False else 'rob1_real_base_link'
			robot_temp = True if robot is False else False

			grab = self.tfBuffer.lookup_transform(temp_base, ch_name+'-GRASP-4', self.rospy.Time(0))
			trans = self.am.trans_convert(self.list_from_trans(grab), [0,0,-0.16,0,0,0,0])

			self.am.hold_assistant(trans[:3], trans[3:], 0.1, robot_temp)#grep pose로 가서 잡기

			self.am.move_current_up(0.01, robot_temp)
			
			self.am.torque_mode(robot_temp, [0,0,1,0,0,0], [0,0,0,0,0,0], tool=True, sleep = 2)

			self.am.sprial_pin(robot_temp)

			self.am.gripper_control(robot_temp, 0)

			self.am.gripper_control(robot, 0)

			self.am.move_current_up(0.1, robot_temp)

			#z로 3cm올리고 tool 기준 z 축 토크 풀고
			#z로 다시 스파이럴(tool 기준 z) 진행


		

		trans = self.tfBuffer.lookup_transform(base_link, ee_link, self.rospy.Time(0))

		trans = self.list_from_trans(trans)

		trans[2] += 0.3 

		# temp_tg, temp_rg = self.am.trans_convert(self.list_from_trans(trans), [0,0,0.3,0, 0, 0, 1])

		self.am.move_motion(trans[:3], trans[3:], 0, robot, c=True)

		# self.am.move_current_up(0.3, robot)

		self.am.init_pose(robot)

		return True, trans_


	def hold_assist(self, rob, part_name, hole, reset=False):



		if rob is True:
			robot = False
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'

		else:
			robot = True
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'


		if reset is True: 
			self.am.gripper_control(robot, 0)
			print "gripper open"		


		point1 = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-1", self.rospy.Time(0))
		point2 = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-2", self.rospy.Time(0))
		
		point1_l = self.list_from_trans(point1, euler=True)
		point2_l = self.list_from_trans(point2, euler=True)

		check = np.linalg.norm(np.array(point1_l[:3]))-np.linalg.norm(np.array(point2_l[:3]))

		# print "hold_assistant"

		if reset is not True:
	
			if check <= 0:
				grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-1", self.rospy.Time(0))

				temp_tg = self.am.trans_convert(self.list_from_trans(grab), [0,0,-0.15,0, 0, 0, 0])

				t = self.am.hold_assistant(temp_tg[:3], temp_tg[3:], 0.1, robot)
				
				if t is False:
					grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-3", self.rospy.Time(0))

					temp_tg= self.am.trans_convert(self.list_from_trans(grab), [0,0,-0.15,0, 0, 0, 0])
				
					t = self.am.hold_assistant(temp_tg[:3], temp_tg[3:], 0.1, robot)

			else:
				grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-2", self.rospy.Time(0))

				temp_tg= self.am.trans_convert(self.list_from_trans(grab), [0,0,-0.15,0, 0, 0, 0])

			
				t = self.am.hold_assistant(temp_tg[:3], temp_tg[3:], 0.1, robot)

				if t is False:
					grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-3", self.rospy.Time(0))

					temp_tg = self.am.trans_convert(self.list_from_trans(grab), [0,0,-0.15,0, 0, 0, 0])
					

					t = self.am.hold_assistant(temp_tg[:3], temp_tg[3:], 0.1, robot)

		else:

			self.am.move_current_up(0.1, robot)

			# if check <= 0:
			# 	grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-1", self.rospy.Time(0))

			# 	grab_l = self.list_from_trans(grab, euler=True)
			
			# 	grab_l[2] += 0.16 #why use?

			# 	self.am.move_motion(grab_l[:3], tf.transformations.quaternion_from_euler(grab_l[3],grab_l[4],grab_l[5]), 0.1, robot)
			# else:
			# 	grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-2", self.rospy.Time(0))

			# 	grab_l = self.list_from_trans(grab, euler=True)

			# 	grab_l[2] += 0.16 #why use?
			# 	self.am.move_motion(grab_l[:3], tf.transformations.quaternion_from_euler(grab_l[3],grab_l[4],grab_l[5]), 0.1, robot)


	def attach_plate(self, robot):

		# waypoint1 
		print "waypoint1"
		raw_input()

		self.am.attach_motion(robot, [-1.34457284609, -1.42656166971, -0.667553901672, -4.17625465016, -1.46750957171, -0.00291997591128])

		# waypoint2

		print "waypoint2"
		raw_input()

		self.am.attach_motion(robot, [-2.56117803255, -1.45668204248, -1.61196041107, -3.21516432385, -0.987765614186, 0.000255584716797])


		print "suction_attach"
		raw_input()

		self.am.suction_attach(robot, [0,0,1,0,0,0], [0,0,-5,0,0,0])

		print "suction on"
		raw_input()

		print "waypoint3"
		raw_input()

		self.am.attach_motion(robot, [-2.48155194918, -1.67284931759, -0.77380657196, -3.83269800762, -1.64082271258, 1.57356262207e-05])

		print "waypoint4"
		raw_input()

		self.am.attach_motion(robot, [-1.0631535689, -1.53731520594, -0.916102409363, -3.82601179699, -1.14120322863, -0.00082236925234])

		print "align"
		raw_input()

		self.am.suction_align(robot)

		print "suction off"
		raw_input()


		 
		# force control 석션 
		# waypoint3 
		# 정렬 


	def trans_from_list(self, l, header, frame_id, euler = False):
		t = geometry_msgs.msg.TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = header
		t.child_frame_id = frame_id
		t.transform.translation.x = l[0]
		t.transform.translation.y = l[1]
		t.transform.translation.z = l[2]
		if euler is not True:
			t.transform.rotation.x = l[3]
			t.transform.rotation.y = l[4]
			t.transform.rotation.z = l[5]
			t.transform.rotation.w = l[6]
		else:
			quat = tf.transformations.quaternion_from_euler(l[3],l[4],l[5])
			t.transform.rotation.x = quat[0]
			t.transform.rotation.y = quat[1]
			t.transform.rotation.z = quat[2]
			t.transform.rotation.w = quat[3]

		return t


	def pose_from_list(self, l, header, frame_id, euler = False):
		t = geometry_msgs.msg.PoseStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = header
		t.pose.position.x = l[0]
		t.pose.position.y = l[1]
		t.pose.position.z = l[2]
		if euler is not True:
			t.pose.orientation.x = l[3]
			t.pose.orientation.y = l[4]
			t.pose.orientation.z = l[5]
			t.pose.orientation.w = l[6]
		else:
			quat = tf.transformations.quaternion_from_euler(l[3],l[4],l[5])
			t.pose.orientation.x = quat[0]
			t.pose.orientation.y = quat[1]
			t.pose.orientation.z = quat[2]
			t.pose.orientation.w = quat[3]

		return t


	def list_from_trans(self, t, euler = False):
		list_trans = []
		
		list_trans.append(t.transform.translation.x)
		list_trans.append(t.transform.translation.y)
		list_trans.append(t.transform.translation.z)
		if euler is not True:
			list_trans.append(t.transform.rotation.x)
			list_trans.append(t.transform.rotation.y)
			list_trans.append(t.transform.rotation.z)
			list_trans.append(t.transform.rotation.w)

		else:
			euler = tf.transformations.euler_from_quaternion([t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
			list_trans.append(euler[0])
			list_trans.append(euler[1])
			list_trans.append(euler[2])

		return list_trans

	# def hand_over_part(self, chname):

	def list_from_pose(self, t, euler = False):
		list_trans = []
		
		list_trans.append(t.position.x)
		list_trans.append(t.position.y)
		list_trans.append(t.position.z)
		if euler is not True:
			list_trans.append(t.orientation.x)
			list_trans.append(t.orientation.y)
			list_trans.append(t.orientation.z)
			list_trans.append(t.orientation.w)

		else:
			euler = tf.transformations.euler_from_quaternion([t.orientation.x,t.orientation.y,t.orientation.z,t.orientation.w])
			list_trans.append(euler[0])
			list_trans.append(euler[1])
			list_trans.append(euler[2])

		return list_trans



	#지금 사용 안함 
	def rot_arrange(self, goal):

		list_trans = []
		temp_trans = []
		euler = tf.transformations.euler_from_quaternion([goal[3],goal[4],goal[5],goal[6]])
		list_trans.append(goal[0])
		list_trans.append(goal[1])
		list_trans.append(goal[2])

		for i in euler:

			if i < 0:
				if abs(i + 3.14) < 0.1:
				 i = -3.1415
				elif abs(i + 1.5707) < 0.1:
				 i = -1.5707
				else:
				 i = 0
			else:
				if abs(i - 3.14) < 0.1:
				 i = 3.1415
				elif abs(i - 1.5707) < 0.1:
				 i = 1.5707
				else:
				 i = 0
			temp_trans.append(i)
		temp = tf.transformations.quaternion_from_euler(temp_trans[0],temp_trans[1],temp_trans[2])

		for j in temp:
			list_trans.append(j)

		return list_trans




def main():
	rospy.init_node('Assembly_Process', anonymous=True)
	ap = Assembly_process(rospy)
	print 'start?'
	raw_input()
	ap.fine_tune_insert_target("hole2-6", False)

if __name__ == '__main__':
	main()