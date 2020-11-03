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


_KHOLECHECKOFFSET = 0.28 # 애들이 정함
#python 규칙 상수는 _대문자


class Assembly_process():

	def __init__(self, ros):
		# self.rospy.init_node('Assembly_Process', anonymous=False)
		# self.params = get_param_from_grasp_yaml()
		# self.motion = Assembly_motion()
		self.rospy = ros
		self.am = Assembly_motion(ros)
		self.tfBuffer = Buffer()
		self.listener = TransformListener(self.tfBuffer)
		self.br = StaticTransformBroadcaster()
		time.sleep(2)
		##########################msg타입 수정 필요##############################3
		# self.pub = self.rospy.Publisher('/camera_op', Float32, queue_size=10)
		# print "go!"


	def elp_camera_client(self, name, robot):
		rob_cam = ''
		if robot is False:
			rob_cam = 'camera_server_1'
		else:
			rob_cam = 'camera_server_2'

		for count in range(10):

			self.rospy.wait_for_service(rob_cam)
			try:
				rob_client = self.rospy.ServiceProxy(rob_cam, cam_Srv)
				basler_data = rob_client(name)
				# print basler_client
				return basler_data
				break
			except self.rospy.ServiceException as e:
				print("Service call failed: %s"%e)

		return False


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
			z_trans = [0,0,0.25,0,0,0,1]
		else:
			rob_camera = 'camera_center_2'
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'
			ro_trans = [0, 0, 0, 0, 0, 0, 1]
			z_trans = [0,0,0.25,0,0,0,1]

		hole_trans = self.tfBuffer.lookup_transform('world', hole_name, self.rospy.Time(0))

		hole_trans = self.qut_from_trans(hole_trans)

		# temp_t, temp_r = self.am.trans_check(hole_trans[:3] + [0,0,0,1], ro_trans)

		# temp_ = temp_t.tolist() + temp_r.tolist()

		temp_ = hole_trans[:3] + ro_trans[3:]

		temp_t, temp_r = self.am.trans_check(temp_, z_trans)

		temp = temp_t.tolist() + temp_r.tolist()

		trans_g = self.tfBuffer.lookup_transform(rob_camera, ee_link, self.rospy.Time(0))

		temp_tg, temp_rg = self.am.trans_check(temp,[0.000, -0.072, 0.058,-0.707, 0.000, 0.000, 0.707])

		base_ = self.tfBuffer.lookup_transform(base_link, 'world', self.rospy.Time(0))

		base_t, base_r = self.am.trans_check(self.qut_from_trans(base_), temp_tg.tolist() + temp_rg.tolist())

		self.am.move_motion(base_t.tolist(), base_r.tolist(), 0, robot, c=True)
			
		#########카메라사용########
		time.sleep(2)
		trans_ = self.client(pa_part, hole_name, robot)


		if move is True:
			if trans_ is not False: 
				self.br.sendTransform(self.tran_from_list(trans_, 'world', 'target_g0'))
				hole_trans = self.tfBuffer.lookup_transform(base_link, 'target_g0', self.rospy.Time(0))
			else:
				hole_trans = self.tfBuffer.lookup_transform(base_link, hole_name, self.rospy.Time(0))


			hole_trans_l = self.list_from_trans(hole_trans)
			

			hole_trans_l[2] += 0.18 #why use?

			if hole_name not in ['hole2-5', 'hole2-6', 'hole3-5', 'hole3-6']:
				hole_trans_l[3] = 3.1415
				hole_trans_l[4] = 0
				hole_trans_l[5] = -1.5707

			# xyz.append(3.1415)
			# xyz.append(0)
			# xyz.append(0)
			# self.am.move_to(xyz, robot)


			self.am.move_motion(hole_trans_l[:3], tf.transformations.quaternion_from_euler(hole_trans_l[3],hole_trans_l[4],hole_trans_l[5]), 0.05, robot)

		else:
			return trans_

		return False

	def client(self, part, hole_name, robot):
		
		for count in range(10):

			result = self.elp_camera_client(hole_name, robot)

			if result is False:
				print "not found"
				break
				return False
				

			else:
				if result.result is False:
					self.am.move_current_to(result.x, result.y, result.z, robot)
					time.sleep(0.5)
				else:
					time.sleep(0.2)
					hole_ = self.tfBuffer.lookup_transform('world', hole_name, self.rospy.Time(0))


					# print "*********************************************************************************************"
					# print "client hole_ : "
					# print hole_

					hole_ = self.qut_from_trans(hole_)

					# print "client1 hole_ : "
					# print hole_

					
					trans_ = self.tfBuffer.lookup_transform('world', result.name, self.rospy.Time(0))

					# trans_ = self.qut_from_trans(trans_)

					trans_ = self.qut_from_trans(trans_)

					trans_[0] += result.x
					trans_[1] += result.y


					###############change 11/04#######################################################################

					# trans_ = self.tfBuffer.lookup_transform('world', 're_target', self.rospy.Time(0))

					# # print "client trans_ : "
					# # print trans_

					# trans_ = self.qut_from_trans(trans_)

					# print "client1 trans_ : "
					# print trans_

					# print "*********************************************************************************************"

					###############change 10/27#######################################################################


					# self.br.sendTransform(self.tran_from_list(trans_[:2]+hole_[2:], 'world', 'target_g'))

					# time.sleep(0.2)

					# # hole_ = self.tfBuffer.lookup_transform(part, hole_name, self.rospy.Time(0))
					# # hole_ = self.qut_from_trans(hole_)

					# # self.br.sendTransform(self.tran_from_list(hole_, part, hole_name))

					# # time.sleep(0.2)

					# hole_ = self.tfBuffer.lookup_transform(hole_name, 'target_g', self.rospy.Time(0))
					# hole_ = self.qut_from_trans(hole_)

					# trans_ = self.tfBuffer.lookup_transform('world', part, self.rospy.Time(0))
					# trans_ = self.qut_from_trans(trans_)

					# trans_t, trans_q = self.am.trans_check(trans_, hole_)

					# self.br.sendTransform(self.tran_from_list(trans_t.tolist()+trans_q.tolist(), 'world', part))
					# time.sleep(0.2)


					# break
					# return -1

					###############change 10/27#######################################################################

					return trans_[:2]+hole_[2:]
						
		return False



	def hand_over_pin_check(self, pin, target_name):

		not_hand_over = ['c122620_1','c122620_2' ,'c122620_3' ,'c122620_4']

		rob1_trans = self.tfBuffer.lookup_transform('rob1_real_base_link', target_name, self.rospy.Time(0))

		rob1_trans_l = self.list_from_trans(rob1_trans)

		rob2_trans = self.tfBuffer.lookup_transform('rob2_real_base_link', target_name, self.rospy.Time(0))

		rob2_trans_l = self.list_from_trans(rob2_trans)

		dist = np.linalg.norm(np.array(rob1_trans_l[:3]))-np.linalg.norm(np.array(rob2_trans_l[:3]))
		if dist > 0:
			if pin in not_hand_over:
				return False

			else:
				self.am.hand_over_pin()
				return True
		else:
			return False

	def list_from_trans(self, t):
		list_trans = []
		euler = tf.transformations.euler_from_quaternion([t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
		list_trans.append(t.transform.translation.x)
		list_trans.append(t.transform.translation.y)
		list_trans.append(t.transform.translation.z)
		list_trans.append(euler[0])
		list_trans.append(euler[1])
		list_trans.append(euler[2])

		return list_trans


	def trans_from_list(self, l, header, frame_id):
		t = geometry_msgs.msg.TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = header
		t.child_frame_id = frame_id
		t.transform.translation.x = l[0]
		t.transform.translation.y = l[1]
		t.transform.translation.z = l[2]
		quat = tf.transformations.quaternion_from_euler(l[3],l[4],l[5])
		t.transform.rotation.x = quat[0]
		t.transform.rotation.y = quat[1]
		t.transform.rotation.z = quat[2]
		t.transform.rotation.w = quat[3]

		return t


	def tran_from_list(self, l, header, frame_id):
		t = geometry_msgs.msg.TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = header
		t.child_frame_id = frame_id
		t.transform.translation.x = l[0]
		t.transform.translation.y = l[1]
		t.transform.translation.z = l[2]
		t.transform.rotation.x = l[3]
		t.transform.rotation.y = l[4]
		t.transform.rotation.z = l[5]
		t.transform.rotation.w = l[6]

		return t


	def qut_from_trans(self, t):
		list_trans = []
		
		list_trans.append(t.transform.translation.x)
		list_trans.append(t.transform.translation.y)
		list_trans.append(t.transform.translation.z)
		list_trans.append(t.transform.rotation.x)
		list_trans.append(t.transform.rotation.y)
		list_trans.append(t.transform.rotation.z)
		list_trans.append(t.transform.rotation.w)

		return list_trans

	# def hand_over_part(self, chname):

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



	def hand_over_part_check(self, ch_name, ch_hole_list, pa_name, pa_hole_list):

		robot = False

		hold = self.tfBuffer.lookup_transform('rob1_real_base_link', ch_name+'-GRASP-3', self.rospy.Time(0))

		hold = self.qut_from_trans(hold)

		success = self.select_part_robot(robot, hold, idx=True)

		if success is False:
			robot = True

			hold = self.tfBuffer.lookup_transform('rob2_real_base_link', ch_name+'-GRASP-3', self.rospy.Time(0))

			hold = self.qut_from_trans(hold)

			success = self.select_part_robot(robot, hold, idx=True)
			if success is not True:
				return -1

		trans = self.tfBuffer.lookup_transform('target', ch_name+'-GRASP-3', self.rospy.Time(0))

		trans_l = self.qut_from_trans(trans)

		temp_t, temp_r = self.am.trans_check(trans_l, [0,0,-0.155,0,0,0,1])

		temp_t = temp_t.tolist()
		temp_r = temp_r.tolist()

		trans_g = self.tfBuffer.lookup_transform('world', 'goal', self.rospy.Time(0))

		# trans_g = self.rot_arrange(self.qut_from_trans(trans_g))

		temp_tg, temp_rg = self.am.trans_check(self.qut_from_trans(trans_g), [0,0,0.05,0.9999997, 0, 0, 0.0007963])

		temp_tg = temp_tg.tolist()
		temp_rg = temp_rg.tolist()

		trans_f, rot_f = self.am.trans_check(temp_tg + temp_rg, temp_t + temp_r)

		trans_f = trans_f.tolist()
		rot_f = rot_f.tolist()

		trans = trans_f + rot_f

		# trans = temp_t + temp_r

		self.br.sendTransform(self.tran_from_list(trans, 'world', "real_goal"))

		time.sleep(1)

		# print "real :   "
		# print self.tfBuffer.lookup_transform('real_goal', 'goal', self.rospy.Time(0))

		success = self.select_part_robot(robot, "real_goal")

		if success is False and robot is False:
			robot = True
			success = self.select_part_robot(robot, "real_goal")
			if success is not True:
				return -1

		hold_check = None

		pa_goal_list = []

		self.hold_assist(robot, pa_name, pa_hole_list[0])

		for i, j in zip(pa_hole_list, range(len(pa_hole_list))):
			select = self.hand_over_hole_check(i)

			# if hold_check is not select:
			# 	self.hold_assist(select, pa_name, i)
			# 	self.hold_assist_reset(select, pa_name, i)
			# 	hold_check = copy.deepcopy(select)
			# 	self.am.init_pose()

			

			trans_ = self.fine_tune_insert_target(pa_name, i, robot, move = False)

			if trans_ is not False:
				self.br.sendTransform(self.tran_from_list(trans_, 'world', 'target_g'+str(j)))
				time.sleep(0.05)
				
			else:
				trans_ = self.tfBuffer.lookup_transform('world', i, self.rospy.Time(0))

				time.sleep(0.05)

				self.br.sendTransform(self.tran_from_list(self.qut_from_trans(trans_), 'world', 'target_g'+str(j)))

				time.sleep(0.05)
				print "*********************************************************************************************"

				print "trans :   "
				print self.tfBuffer.lookup_transform('world', 'target_g'+str(j), self.rospy.Time(0))

				print "*********************************************************************************************"

			pa_goal_list.append('target_g'+str(j))



		self.send_tf(pa_name, pa_goal_list, ch_name, ch_hole_list)

		trans_g = self.tfBuffer.lookup_transform('world', 'goal', self.rospy.Time(0))

		# trans_g = self.rot_arrange(self.qut_from_trans(trans_g))

		temp_tg, temp_rg = self.am.trans_check(self.qut_from_trans(trans_g), [0,0,-0.05,0.9999997, 0, 0, 0.0007963])

		temp_tg = temp_tg.tolist()
		temp_rg = temp_rg.tolist()

		trans_f, rot_f = self.am.trans_check(temp_tg + temp_rg, temp_t + temp_r)

		trans_f = trans_f.tolist()
		rot_f = rot_f.tolist()

		trans = trans_f + rot_f

		# trans = temp_t + temp_r

		self.br.sendTransform(self.tran_from_list(trans, 'world', "real_goal"))

		time.sleep(1)

		# print self.tfBuffer.lookup_transform('real_goal', 'goal', self.rospy.Time(0))
			
		return robot


	def hand_over_hole_check(self, target_name):

		rob1_trans = self.tfBuffer.lookup_transform('rob1_real_base_link', target_name, self.rospy.Time(0))

		rob1_trans_l = self.list_from_trans(rob1_trans)

		rob2_trans = self.tfBuffer.lookup_transform('rob2_real_base_link', target_name, self.rospy.Time(0))

		rob2_trans_l = self.list_from_trans(rob2_trans)

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
			trans = self.tfBuffer.lookup_transform(base_frame, goal, self.rospy.Time(0)) 
			trans = self.qut_from_trans(trans)

			success = self.am.move_check(trans[:3], trans[3:], 0, robot, c=True)
		else: 
			trans = goal
			trans[2] += 0.155 
			success = self.am.move_check(trans[:3], trans[3:], 0.1, robot, c=True)

		return success

	def grab_pin(self, pin_name):#asm_child_msg, is_moved):
		# grasp = self.make_grasp_msg(asm_child_msg.pin, asm_child_msg.pose)
		# self.motion.pick_up_pin(grasp)
		self.am.pick_up_pin(pin_name)


	def send_tf(self, pa_name, pa_hole_list, ch_name, ch_hole_list):

		#self.am.init_pose()


		goal_list = []
		target_list = []

		goal = [0,0,0,0,0,0]
		target = [0,0,0,0,0,0]


		

		for i, j in zip(pa_hole_list, ch_hole_list):
			goal_list.append(self.tfBuffer.lookup_transform(pa_name, i, self.rospy.Time(0)))
			target_list.append(self.tfBuffer.lookup_transform(ch_name, j, self.rospy.Time(0)))
		
		for i, j in zip(goal_list, target_list):
			temp_g = self.list_from_trans(i)
			temp_t = self.list_from_trans(j)
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


		self.br.sendTransform(self.trans_from_list(goal, pa_name, "goal"))
		self.br.sendTransform(self.trans_from_list(target, ch_name, "target"))

		time.sleep(1)


	def grab_part(self, robot, ch_name):

		Path = "/home/care/ur_ws/src/Assembly_UR5e_workspace/assembly_robot_control/ulol_tf/object_description/chair_meshes/"

		if robot is False:
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
		else:
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'

		trans = self.tfBuffer.lookup_transform(base_link, ch_name+'-GRASP-3', self.rospy.Time(0))

		trans = self.qut_from_trans(trans)
		
		trans[2] += 0.152 #why use?

		self.am.hold_assistant(trans[:3], trans[3:], 0.1, robot)

		trans_ = self.tfBuffer.lookup_transform(ee_link, ch_name, self.rospy.Time(0))

		self.am.attach(robot, ch_name, Path + 'chair part' + ch_name[-1] + ".SLDPRT.STL")

		time.sleep(0.5)

		# trans_ = self.qut_from_trans(trans_)

		self.am.move_motion(trans[:3], trans[3:], 0.1, robot)

		trans = self.tfBuffer.lookup_transform(base_link, 'real_goal', self.rospy.Time(0))

		trans = self.qut_from_trans(trans)

		self.am.move_motion(trans[:3], trans[3:], 0, robot, c=True)

		return trans_

		
	def insert_spiral_pin_motion(self, robot, num_of_trial=5):
		# spiral() 실행, 성공할 때까지 num_of_trial 만큼 반복
		# target pose와 grasp_config.yaml 의 데이터를 합쳐서 approach, retreat도 결정 
		# for i in range(num_of_trial):
		# 	if self.am.sprial_motion():
		# 		break
		self.am.sprial_pin(robot)

	def insert_spiral_part_motion(self, robot, trans_,num_of_trial=5):
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


		self.am.sprial_pin(robot)#, 1)
		self.am.dettach(robot, trans_.child_frame_id)

		trans = self.tfBuffer.lookup_transform('world', trans_.header.frame_id, self.rospy.Time(0))

		trans, rot = self.am.trans_check(self.qut_from_trans(trans), self.qut_from_trans(trans_))

		self.br.sendTransform(self.tran_from_list(trans.tolist()+ rot.tolist(), 'world', trans_.child_frame_id))

		trans = self.tfBuffer.lookup_transform(base_link, ee_link, self.rospy.Time(0))

		trans = self.qut_from_trans(trans)

		trans[2] += 0.3 

		# temp_tg, temp_rg = self.am.trans_check(self.qut_from_trans(trans), [0,0,0.3,0, 0, 0, 1])

		self.am.move_motion(trans[:3], trans[3:], 0, robot, c=True)

		# self.am.move_current_up(0.3, robot)

		self.am.init_pose(robot)

	def insert_part_motion(sorted_insert_target_poses):
		## ??? 
		pass

	def sort_insert_target(self, asm_parents_msg):
		groups = dict()
		singles = []
		for parent in len(asm_parents_msg):
			if parent.group_name is not '':
				if parent.group_name in gruops:
					groups[parent.group_name].append(parent.target)
				else:
					groups[parent.group_name] = [parent.target]
			else:
				singles.append(parent.target)

		insert_sorted_poses = self.sort_priority(groups, singles) 
		return insert_sorted_poses

	def sort_priority(groups, singles):
		# priority는 single 인지 group 인지를 기준으로 설정
		# 추후 child를 잡고 있는 위치 값도 들어가야 함(무게중심의 기준값으로 친다)
		pass

	def make_grasp_msg(self, part_name, child_frame_pose):
		# child_frame_pose가 들어오면, part_name을 pick_config.yaml에서 찾아서
		# 해당하는 part_grasp_target_pose 값과 child_frame_pose를 합쳐서 grasping pose(robot base frame 기준) 결정
		# child_frame_pose가 들어오지 않으면, pick_config.yaml에서 
		# common_area_grasp_target_pose(world frame 기준)로 grasping pose 결정
		global_child_pose = child_frame_pose
		self.params[part_name].target_pose
		self.params[part_name].offset		
		return grasp


	def hold_assist(self, rob, part_name, hole):

		if rob is True:
			robot = False
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
		else:
			robot = True
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'


		# point1 = self.tfBuffer.lookup_transform(part_name + "-GRASP-1", hole, self.rospy.Time(0))
		# point2 = self.tfBuffer.lookup_transform(part_name + "-GRASP-2", hole, self.rospy.Time(0))

		point1 = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-1", self.rospy.Time(0))
		point2 = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-2", self.rospy.Time(0))
		
		point1_l = self.list_from_trans(point1)
		point2_l = self.list_from_trans(point2)

		check = np.linalg.norm(np.array(point1_l[:3]))-np.linalg.norm(np.array(point2_l[:3]))

		print "hold_assistant"
	
		if check <= 0:
			grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-1", self.rospy.Time(0))

			temp_tg, temp_rg = self.am.trans_check(self.qut_from_trans(grab), [0,0,-0.16,0, 0, 0, 0])

			temp_tg = temp_tg.tolist()
			temp_rg = temp_rg.tolist()


			t = self.am.hold_assistant(temp_tg, temp_rg, 0.1, robot)
			
			if t is False:
				grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-3", self.rospy.Time(0))

				temp_tg, temp_rg = self.am.trans_check(self.qut_from_trans(grab), [0,0,-0.16,0, 0, 0, 0])

				temp_tg = temp_tg.tolist()
				temp_rg = temp_rg.tolist()
			
				t = self.am.hold_assistant(temp_tg, temp_rg, 0.1, robot)

		else:
			grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-2", self.rospy.Time(0))

			temp_tg, temp_rg = self.am.trans_check(self.qut_from_trans(grab), [0,0,-0.16,0, 0, 0, 0])

			temp_tg = temp_tg.tolist()
			temp_rg = temp_rg.tolist()
		
			t = self.am.hold_assistant(temp_tg, temp_rg, 0.1, robot)

			if t is False:
				grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-3", self.rospy.Time(0))

				temp_tg, temp_rg = self.am.trans_check(self.qut_from_trans(grab), [0,0,-0.16,0, 0, 0, 0])

				temp_tg = temp_tg.tolist()
				temp_rg = temp_rg.tolist()

				t = self.am.hold_assistant(temp_tg, temp_rg, 0.1, robot)



	def hold_assist_reset(self, rob, part_name, hole):
		self.am.gripper_control(rob, 0)


		if rob is True:
			robot = False
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
		else:
			robot = True
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'


		# point1 = self.tfBuffer.lookup_transform(part_name + "-GRASP-1", hole, self.rospy.Time(0))
		# point2 = self.tfBuffer.lookup_transform(part_name + "-GRASP-2", hole, self.rospy.Time(0))

		point1 = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-1", self.rospy.Time(0))
		point2 = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-2", self.rospy.Time(0))
		
		point1_l = self.list_from_trans(point1)
		point2_l = self.list_from_trans(point2)

		check = np.linalg.norm(np.array(point1_l[:3]))-np.linalg.norm(np.array(point2_l[:3]))

		print check
	
		if check <= 0:
			grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-1", self.rospy.Time(0))

			grab_l = self.list_from_trans(grab)
		
			grab_l[2] += 0.16 #why use?

			self.am.move_motion(grab_l[:3], tf.transformations.quaternion_from_euler(grab_l[3],grab_l[4],grab_l[5]), 0.1, robot)
		else:
			grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-2", self.rospy.Time(0))

			grab_l = self.list_from_trans(grab)
			print "aa"
			grab_l[2] += 0.16 #why use?
			self.am.move_motion(grab_l[:3], tf.transformations.quaternion_from_euler(grab_l[3],grab_l[4],grab_l[5]), 0.1, robot)


	def make_insert_msg(part_name, child_frame_pose):
		pass

def main():
	rospy.init_node('Assembly_Process', anonymous=True)
	ap = Assembly_process(rospy)
	print 'start?'
	raw_input()
	ap.fine_tune_insert_target("hole2-6", False)

if __name__ == '__main__':
	main()