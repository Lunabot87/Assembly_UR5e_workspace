#!/usr/bin/env python
#-*- coding:utf-8 -*-

from std_msgs.msg import * 
from Assembly_Motion_test import Assembly_motion
from tf import *
import numpy as np


_KHOLECHECKOFFSET = 0.28 # 애들이 정함
#python 규칙 상수는 _대문자


class Assembly_process():

	def __init__(self, ros):
		# self.rospy.init_node('Assembly_Process', anonymous=False)
		# self.params = get_param_from_grasp_yaml()
		# self.motion = Assembly_motion()
		self.rospy = ros
		self.am = Assembly_motion()
		self.listener = TransformListener()
		##########################msg타입 수정 필요##############################3
		self.pub = self.rospy.Publisher('/camera_op', Float32, queue_size=10)
		print "go!"

	def fine_tune_insert_target(self, part_name, robot):
		# target_pose[PoseStamped] : 핀을 꽂은 상태에서 eef의 목표 값
		# kHoleCheckOffset
		# 7/22 적용
		# xyz, rpy = self.listener.lookupTransform('/world', part_name, self.rospy.Time(0))

		self.am.camera_pose(robot)

		xyz, rpy = self.listener.lookupTransform('/world', part_name, self.rospy.Time(0))

		xyz[2] += 0.2
		if robot is False:
			xyz.append(-1.5707)
			xyz.append(0)
			xyz.append(-3.1415)
		else:
			xyz.append(1.5707)
			xyz.append(3.1415)
			xyz.append(-3.1415)

		self.am.move_to(xyz, robot)
		self.pub.publish(1)

		xyz, rpy = self.listener.lookupTransform('/world', part_name, self.rospy.Time(0))
		

		xyz[2] += 0.4

		xyz.append(3.1415)
		xyz.append(0)
		xyz.append(0)
		self.am.move_to(xyz, robot)

		#return target_pose

	def hand_over_pin_check(self, target_name):

		rob1_xyz, rob1_rpy = self.listener.lookupTransform('/rob1_real_base_link', target_name, self.rospy.Time(0))
		rob2_xyz, rob2_rpy = self.listener.lookupTransform('/rob2_real_base_link', target_name, self.rospy.Time(0))

		dist = np.linalg.norm(np.array(rob1_xyz))-np.linalg.norm(np.array(rob2_xyz))
		if dist > 0:
			self.am.hand_over_pin()
			return True
		else:
			return False


	def hand_over_part(self, insert_target_pose, asm_msg):
		# 1. 낄 수 있는지 2. 들 수 있는지 확인
		# child part를 낄 수 있는지 확인해서 못끼면, 로봇을 변경
		# 바뀐 로봇이 child part를 들 수 있는지 확인해서 못들면, child part를 옮긴다.
		# 끼는 target 위치는 insert 할 위치
		# 드는 target 위치는 child part의 위치
		# 작업하는 로봇이 바뀐 경우 True 반환, 그대로인 경우 False 반환
		rob = rob1
		# rob1이 낄 수 있거나 rob2가 낄수 있거나, 둘다 안되는 경우는 발생 x 가정
		if not check_reachability(insert_target_pose, rob):
			rob = rob2 
		if not check_reachability(asm_msg.child.pose, rob):
			pass_part_to_other_rob(asm_msg.child.pose, COMMON_AREA_TARGET_POSE, rob)
			return True

		# check_reachability, pass_part_to_other_rob 생성 필요
		return False


	def grab_pin(self, pin_name):#asm_child_msg, is_moved):
		# grasp = self.make_grasp_msg(asm_child_msg.pin, asm_child_msg.pose)
		# self.motion.pick_up_pin(grasp)
		self.am.pick_up_pin(pin_name)


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

	def make_insert_msg(part_name, child_frame_pose):
		pass

# def main():
# 	ap = Assembly_process()
# 	ap.am.pick_up_pin("1")
# 	ap.am.hand_over_pin()

# if __name__ == '__main__':
# 	main()