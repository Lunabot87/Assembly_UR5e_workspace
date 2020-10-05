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


	def fine_tune_insert_target(self, hole_name, robot):
		# target_pose[PoseStamped] : 핀을 꽂은 상태에서 eef의 목표 값
		# kHoleCheckOffset
		# 7/22 적용
		# xyz, rpy = self.listener.lookupTransform('/world', part_name, self.rospy.Time(0))

		# self.am.camera_pose(robot)

		rob_frame = ''
		if robot is False:
			rob_camera = 'camera_center_1'
			ee_link = 'rob1_real_ee_link'
			base_link= 'rob1_real_base_link'
		else:
			rob_camera = 'camera_center_2'
			ee_link = 'rob2_real_ee_link'
			base_link= 'rob2_real_base_link'

		hole_trans = self.tfBuffer.lookup_transform('world', hole_name, self.rospy.Time(0))

		hole_trans_l = self.list_from_trans(hole_trans)

		of_trans = self.tfBuffer.lookup_transform(ee_link, rob_camera, self.rospy.Time(0))

		of_trans_l = self.list_from_trans(of_trans)

		if robot is False:
			hole_trans_l[0] += of_trans_l[0]
			hole_trans_l[1] += of_trans_l[1]
		else:
			hole_trans_l[0] -= of_trans_l[0]
			hole_trans_l[1] -= of_trans_l[1]

		hole_trans_l[2] += 0.25
		if robot is False:
			hole_trans_l[3] = -1.5707
			hole_trans_l[4] = 0
			hole_trans_l[5] = -3.1415
		else:
			hole_trans_l[3] = 1.5707
			hole_trans_l[4] = 3.1415
			hole_trans_l[5] = -3.1415

		self.am.camera_pose(robot)

		self.am.move_to(hole_trans_l, robot)
			

		#########error 수정필요########
		# for count in range(10):

		# 	result = self.elp_camera_client(hole_name, robot)
		# 	if result is False:
		# 		print "not found"
		# 		break
		# 	else:
		# 		if result.result is False:
		# 			self.am.move_current_to(result.x, result.y, result.z, robot)

		# 		else:
		# 			# xyz, rpy = self.tfBuffer.lookup_transform'world', 'target', self.rospy.Time(0))
		# 			# while not self.listener.frameExists('re_target'):
		# 			# 	print "nonono"
		# 			time.sleep(2)
		# 			break
		
		hole_trans = self.tfBuffer.lookup_transform(base_link, hole_name, self.rospy.Time(0))

		hole_trans_l = self.list_from_trans(hole_trans)
		
		hole_trans_l[2] += 0.16 #why use?

		# xyz.append(3.1415)
		# xyz.append(0)
		# xyz.append(0)
		# self.am.move_to(xyz, robot)


		self.am.move_motion(hole_trans_l[:3], tf.transformations.quaternion_from_euler(hole_trans_l[3],hole_trans_l[4],hole_trans_l[5]), 0.1, robot)

		#return target_pose



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


	def hand_over_part_check(self, ch_name):

		trans = self.tfBuffer.lookup_transform('rob1_real_base_link', ch_name+'-GRASP-1', self.rospy.Time(0))

		trans_l = self.qut_from_trans(trans)

		trans_l[2] = 0.16

		self.am.move_motion(trans_l[:3], trans_l[3:], 0.1, False)

		self.am.move_motion(trans_l[:3], trans_l[3:], 0, False)

		time.sleep(1)

		trans_t = self.tfBuffer.lookup_transform('target', 'rob1_real_ee_link', self.rospy.Time(0))

		trans_g = self.tfBuffer.lookup_transform('chair_part6', 'goal', self.rospy.Time(0))


		temp_t, temp_r = self.am.trans_check(self.qut_from_trans(trans_g), [0,0,-0.15,0.9999997, 0, 0, 0.0007963])

		temp_t = temp_t.tolist()
		temp_r = temp_r.tolist()

		trans_f, rot_f = self.am.trans_check(temp_t + temp_r, self.qut_from_trans(trans_t))

		trans_f = trans_f.tolist()
		rot_f = rot_f.tolist()

		trans = trans_f + rot_f

		# trans = temp_t + temp_r

		self.br.sendTransform(self.tran_from_list(trans, 'chair_part6', "real_goal"))

		time.sleep(1)

		trans = self.tfBuffer.lookup_transform('rob1_real_base_link', 'real_goal', self.rospy.Time(0))

		trans = self.qut_from_trans(trans)

		# trans[2] = 0.16

		self.am.move_motion(trans[:3], trans[3:], 0, False)

		print "fin"

		# rob2 = self.tfBuffer.lookup_transform('rob2_real_base_link', 'target', self.rospy.Time(0))

		# rob = rob1
		# # rob1이 낄 수 있거나 rob2가 낄수 있거나, 둘다 안되는 경우는 발생 x 가정
		# if not check_reachability(insert_target_pose, rob):
		# 	rob = rob2 
		# if not check_reachability(asm_msg.child.pose, rob):
		# 	pass_part_to_other_rob(asm_msg.child.pose, COMMON_AREA_TARGET_POSE, rob)
		# 	return True
		#######################################
		# check_reachability, pass_part_to_other_rob 생성 필요
		return False


	def grab_pin(self, pin_name):#asm_child_msg, is_moved):
		# grasp = self.make_grasp_msg(asm_child_msg.pin, asm_child_msg.pose)
		# self.motion.pick_up_pin(grasp)
		self.am.pick_up_pin(pin_name)


	def send_tf(self, pa_name, pa_hole_list, ch_name, ch_hole_list):
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


		self.br.sendTransform(self.trans_from_list(goal, pa_name, "goal"))
		self.br.sendTransform(self.trans_from_list(target, ch_name, "target"))

		time.sleep(1)


	def grab_part(self, asm_child_msg, is_moved):
		if is_moved is True:
			grasp = self.make_grasp_msg(asm_child_msg.part)
		else:
			grasp = self.make_grasp_msg(asm_child_msg.part, asm_child_msg.pose)
		self.motion.pick_up(grasp)

	def insert_spiral_motion(self, robot, num_of_trial=5):
		# spiral() 실행, 성공할 때까지 num_of_trial 만큼 반복
		# target pose와 grasp_config.yaml 의 데이터를 합쳐서 approach, retreat도 결정 
		# for i in range(num_of_trial):
		# 	if self.am.sprial_motion():
		# 		break
		self.am.sprial_pin(robot)

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


		point1 = self.tfBuffer.lookup_transform(part_name + "-GRASP-1", hole, self.rospy.Time(0))
		point2 = self.tfBuffer.lookup_transform(part_name + "-GRASP-2", hole, self.rospy.Time(0))
		
		point1_l = self.list_from_trans(point1)
		point2_l = self.list_from_trans(point2)

		check = np.linalg.norm(np.array(point1_l[:3]))-np.linalg.norm(np.array(point2_l[:3]))

		print check
	
		if check >= 0:
			grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-1", self.rospy.Time(0))

			grab_l = self.list_from_trans(grab)
		
			grab_l[2] += 0.16 #why use?

			self.am.hold_assistant(grab_l[:3], tf.transformations.quaternion_from_euler(grab_l[3],grab_l[4],grab_l[5]), 0.1, robot)
		else:
			grab = self.tfBuffer.lookup_transform(base_link, part_name + "-GRASP-2", self.rospy.Time(0))

			grab_l = self.list_from_trans(grab)
			print "aa"
		
			grab_l[2] += 0.16 #why use?
			self.am.hold_assistant(grab_l[:3], tf.transformations.quaternion_from_euler(grab_l[3],grab_l[4],grab_l[5]), 0.1, robot)


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


		point1 = self.tfBuffer.lookup_transform(part_name + "-GRASP-1", hole, self.rospy.Time(0))
		point2 = self.tfBuffer.lookup_transform(part_name + "-GRASP-2", hole, self.rospy.Time(0))
		
		point1_l = self.list_from_trans(point1)
		point2_l = self.list_from_trans(point2)

		check = np.linalg.norm(np.array(point1_l[:3]))-np.linalg.norm(np.array(point2_l[:3]))

		print check
	
		if check >= 0:
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