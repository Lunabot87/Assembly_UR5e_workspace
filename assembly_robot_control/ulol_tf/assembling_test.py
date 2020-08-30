#!/usr/bin/env python

import rospy
import tf
import numpy
import random
import time


from math import pi
from copy import deepcopy
from moveit_commander.conversions import list_to_pose,pose_to_list

import move_group_wrapper_0801 as MGW
import Part_ulol	# PART CLASS
from Part_Pin import assembly_data


def pose_compare(target_pose,cur_pose):
	target_pose_quat = Part_ulol.make_orientation_list(target_pose.orientation)
	target_pose_mat = Part_ulol.quaternion_matrix(target_pose_quat)
	target_pose_trans = Part_ulol.make_position_list(target_pose.position)
	target_pose_mat[0:3,3] = target_pose_trans

	cur_pose_quat = Part_ulol.make_orientation_list(cur_pose.orientation)
	cur_pose_mat = Part_ulol.quaternion_matrix(cur_pose_quat)
	cur_pose_trans = Part_ulol.make_position_list(cur_pose.position)
	cur_pose_mat[0:3,3] = cur_pose_trans

	target_pose_round = numpy.around(target_pose_mat,2)
	cur_pose_round = numpy.around(cur_pose_mat,2)

	is_equal = target_pose_round == cur_pose_round
	for i in range(4):
		for j in range(4):
			if is_equal[i,j] == False:
				print '\n',target_pose_round,'\n',cur_pose_round
				return False
	return True
def Is_part_on_proper_pose(part_tf_list, target_part_name, proper_pose = Part_ulol.part_init_pose):
	part_num = Part_ulol.part_name.index(target_part_name)
	cur_pose = part_tf_list[part_num]['origin']
	proper_pose = proper_pose[part_num].pose
	if target_part_name in Part_ulol.part_name:
		if pose_compare(cur_pose,proper_pose):
			print "GOOD POSE"
			return True
		else:
			print "BAD POSE"
			return False
	elif target_part_name in Part_ulol.pin_name:
		print "[ERROR] Select Part : ",part_name,"is a Pin"
		return True
	else:
		print "[ERROR] WRONG NAME"
		return True
def which_rob_is_near_to(gp_pose, part_name = None, cur_rob = None):
	if part_name == 'chair_part6':
		criteria = 0.30
	elif part_name =='chair_part5':
		criteria = -0.20
	else:
		criteria = 0.1

	if cur_rob == 'rob1':
		criteria -= 0.15

	if gp_pose.position.y > criteria:
		return 'rob1'
	else:
		return 'rob2'

def attach(CLASS_SET,rob_name,attach_list):

	ROB = CLASS_SET[rob_name]
	PART_SCENE = CLASS_SET['tf']

	if rob_name == 'rob1':
		grasping_group = 'rob1_hand'
		eef_link = 'rob1_tcp_gripper_closed'
	elif rob_name == 'rob2':
		grasping_group = 'rob2_hand'
		eef_link = 'rob2_tcp_gripper_closed'
	# touch_links = ROB.moveit_commander.RobotCommander().get_link_names(group=grasping_group)

	for attaching_part_name in attach_list['part']:
		mesh_file = Part_ulol.part_file[Part_ulol.part_name.index(attaching_part_name)]
		PART_SCENE.scene.attach_mesh(eef_link, attaching_part_name, filename = mesh_file, size = (1,1,1))

	for attaching_pin_name in attach_list['pin']:
		temp_pin_name = attaching_pin_name.split('-')[0]
		mesh_file = Part_ulol.part_file[Part_ulol.pin_name.index(temp_pin_name)]
		PART_SCENE.scene.attach_mesh(eef_link, attaching_pin_name, filename = mesh_file, size = (1,1,1))
def detach(CLASS_SET,rob_name,attach_list):
	PART_SCENE = CLASS_SET['tf']

	if rob_name == 'rob1':
		eef_link = 'rob1_tcp_gripper_closed'
	elif rob_name == 'rob2':
		eef_link = 'rob2_tcp_gripper_closed'

	for attaced_part_name in attach_list['part']:
		PART_SCENE.scene.remove_attached_object(eef_link, attaced_part_name)

	for attaced_pin_name in attach_list['pin']:
		PART_SCENE.scene.remove_attached_object(eef_link, attaced_pin_name)
def pick_the_mesh_up(CLASS_SET, rob_name, target_mesh_name,tag_offset = None, holding = False):
	print "\n[PICK THE MESH UP]==================================================="
	if rob_name == 'rob1':
		rob_base = 'rob1_real_base_link'
		sub_rob = 'rob2'
	elif rob_name == 'rob2':
		rob_base = 'rob2_real_base_link'
		sub_rob = 'rob1'
	PART_SCENE = CLASS_SET['tf']
	if holding == False:
		ROB = CLASS_SET[sub_rob]
		ROB.go_to_initial_pose()
	ROB = CLASS_SET[rob_name]
	ROB.go_to_initial_pose()

	if target_mesh_name in Part_ulol.part_name:
		grasp_point = target_mesh_name+'-GRASP-'+str(tag_offset+1)
		attach_list = PART_SCENE.get_attach_list(target_mesh_name)

	elif target_mesh_name in Part_ulol.pin_name:
		grasp_point = target_mesh_name+'-'+str(tag_offset+1)
		target_pin_name = target_mesh_name+'-'+str(tag_offset+1)
		attach_list = PART_SCENE.get_attach_list(target_pin_name)

	else:
		print "WRONG NAME"
		return

	(g_trans, g_rot) = CLASS_SET['listener'].lookupTransform(rob_base, grasp_point, rospy.Time(0))

	offset = 0.1

	(pg_trans, pg_rot) = ROB._grasp_to_pregrasp(g_trans, g_rot, offset)
	(result0, _) = ROB._get_best_ik_plan(g_trans, g_rot)
	ROB.go_to_pose_goal(pg_trans, pg_rot, result0)
	ROB.go_linear_to_pose_goal(g_trans, g_rot, result0)
	
	print "ATTACH = ", attach_list
	attach(CLASS_SET,rob_name,attach_list)
	ROB.go_linear_to_pose_goal(pg_trans, pg_rot, result0)
	print "===================================================[PICK THE MESH UP]\n"
def put_the_part_down(CLASS_SET, rob_name, target_part_name, goal_pose, offset = 0.1):
	print "\n[PUT THE PART DOWN]=================================================="
	if rob_name == 'rob1':
		rob_base = 'rob1_real_base_link'
	elif rob_name == 'rob2':
		rob_base = 'rob2_real_base_link'
	ROB = CLASS_SET[rob_name]
	PART_SCENE = CLASS_SET['tf']
	goal_pose_name = 'GOAL_POSE'
	PART_SCENE.send_temporary_TF(goal_pose, goal_pose_name)
	# print "GOAL_POSE_SENT, ENTER",
	# raw_input()
	while not CLASS_SET['listener'].frameExists('GOAL_POSE'):
		pass
	(g_trans, g_rot) = CLASS_SET['listener'].lookupTransform(rob_base, goal_pose_name, rospy.Time(0))
	(pg_trans, pg_rot) = ROB._grasp_to_pregrasp(g_trans, g_rot, offset)
	(result0, _) = ROB._get_best_ik_plan(g_trans, g_rot)
	ROB.go_to_pose_goal(pg_trans, pg_rot, result0)

	ROB.go_linear_to_pose_goal(g_trans, g_rot, result0)
	# ROB.go_to_pose_goal(pg_trans, pg_rot, result0)
	attach_list = PART_SCENE.get_attach_list(target_part_name)
	detach(CLASS_SET,rob_name,attach_list)
	ROB.go_linear_to_pose_goal(pg_trans, pg_rot, result0)
	PART_SCENE.init_attach_list()
	ROB.go_to_initial_pose()
	print "==================================================[PUT THE PART DOWN]\n"


def hold_part(CLASS_SET,target_part_name, hold_point_num, rob_name = 'rob2'):
	if not hold_point_num == None:
		print "\n[HOLD PART]=========================================================="
		ROB = CLASS_SET[rob_name]
		if rob_name == 'rob1':
			rob_base = 'rob1_real_base_link'
		elif rob_name == 'rob2':
			rob_base = 'rob2_real_base_link'
		PART_SCENE = CLASS_SET['tf']

		if not target_part_name in Part_ulol.part_name:
			print "hold_part : [WRONG_PART_NAME']"
			return
		else:
			hold_point = target_part_name+'-GRASP-'+str(hold_point_num+1)
			print "hold_point =",hold_point
			offset = 0.1
			time.sleep(1)
			(h_trans, h_rot) = CLASS_SET['listener'].lookupTransform(rob_base, hold_point, rospy.Time(0))
			
			(ph_trans, ph_rot) = ROB._grasp_to_pregrasp(h_trans, h_rot, offset)
			(result0, _) = ROB._get_best_ik_plan(h_trans, h_rot)
			ROB.go_to_pose_goal(ph_trans, ph_rot, result0)
			ROB.go_linear_to_pose_goal(h_trans, h_rot, result0)

		print "==========================================================[HOLD PART]\n"
def move_part_to_place_pose(CLASS_SET, target_part_name, proper_pose = Part_ulol.part_init_pose, gp = None, rob = None, holding = True):
	print "\n[MOVE PART TO PLACE POSE]============================================"
	PART_SCENE = CLASS_SET['tf']
	print target_part_name,
	if Is_part_on_proper_pose(PART_SCENE.TF_List,target_part_name,proper_pose):
		print "Already good pose"
	else:
		if proper_pose == Part_ulol.part_init_pose:
			print "===>[INIT_POSE]"
		elif proper_pose == Part_ulol.part_ready_pose:
			print "===>[READY_POSE]"
		elif proper_pose == Part_ulol.part_spare_pose:
			print "===>[SPARE_POSE]"

		part_num = Part_ulol.part_name.index(target_part_name)
		num_of_gp = Part_ulol.grasp_in_part[part_num]
		gp_list = PART_SCENE.GP_List[part_num]['pose']
		dist_to_table_org=[]
		
		# the nearist GP select
		if gp == None:
			for i in range(num_of_gp):
				gp_position = gp_list[i].position
				gp_point = numpy.array([gp_position.x,gp_position.y-0.05,0])
				dist = numpy.linalg.norm(gp_point)
				dist_to_table_org.append(deepcopy(dist))

			cur_gp_num = dist_to_table_org.index(min(dist_to_table_org))
		else:
			cur_gp_num = gp
		print "GRASP_POINT-",cur_gp_num+1
		cur_gp_pose = gp_list[cur_gp_num]

		# proper selected gp pose of proper org pose
		proper_org = proper_pose[part_num].pose
		tf_offset = Part_ulol.GP.grasping_pose[part_num][cur_gp_num]
		proper_gp_pose = PART_SCENE.get_TF_pose(proper_org, tf_offset)

		rob_near_to_cur = which_rob_is_near_to(cur_gp_pose, target_part_name,'rob2')
		rob_near_to_prop = which_rob_is_near_to(proper_gp_pose, target_part_name, rob_near_to_cur)
		if rob == None:
			if not rob_near_to_cur == which_rob_is_near_to(cur_gp_pose):
					if rob_near_to_cur == 'rob1':
						rob_near_to_cur = 'rob2'
					elif rob_near_to_cur == 'rob2':
						rob_near_to_cur = 'rob1'
		else:
			rob_near_to_cur = rob
			rob_near_to_prop = rob
		print "SELECTED ROBOT =",rob_near_to_cur
		pick_the_mesh_up(CLASS_SET, rob_near_to_cur, target_part_name, cur_gp_num, holding)

		if rob_near_to_prop == rob_near_to_cur:
			print "SAME ROB"
			put_the_part_down(CLASS_SET, rob_near_to_cur, target_part_name, proper_gp_pose)
		else:
			print "DIFF ROB"
			temp_gp_pose = deepcopy(cur_gp_pose)
			temp_gp_pose.position.x = 0
			if rob_near_to_cur == 'rob1':				
				temp_gp_pose.position.y = 0
			elif rob_near_to_cur == 'rob2':
				temp_gp_pose.position.y = 0.1
			ROB = CLASS_SET[rob_near_to_cur]
			put_the_part_down(CLASS_SET, rob_near_to_cur, target_part_name, temp_gp_pose)
			move_part_to_place_pose(CLASS_SET,target_part_name,proper_pose)
	
		while PART_SCENE.TF_List[part_num]['origin'] == []:
			pass
	print "[MOVE PART TO PLACE POSE]============================================\n"
def grab_pin(CLASS_SET, pin_name,pin_tag):
	print "\n[GRAB PIN]==========================================================="
	pick_the_mesh_up(CLASS_SET,'rob1',pin_name,pin_tag,True)
	print "===========================================================[GRAB PIN]\n"
def insert_pin(CLASS_SET, target_pin, pin_tag, target_part, hole_num):# target_pin : ex)pin101340-2
	print "\n[INSERT_PIN]========================================================="
	if not target_part in Part_ulol.part_name:
		print "[insert_pin] : WRONG_TARGET_PART"
		return
	else:
		target_pin_name = target_pin+'-'+str(pin_tag+1)
		print "[Insert",target_pin_name,"-",hole_num,"to",target_part,"-",hole_num,"]"
		PART_SCENE = CLASS_SET['tf']
		part_num = Part_ulol.part_name.index(target_part)

		print "INSERT_PIN : part_num =",part_num

		if not hole_num in range(Part_ulol.holes_in_part[part_num]):
			print "[insert_pin] : WRONG_HOLE_NUMBER"
		else:
			if part_num in [1,2]:
				if hole_num is 4:
					hold_point_offset = 3	
					if part_num == 2:
						if not Is_part_on_proper_pose(PART_SCENE.TF_List,target_part,Part_ulol.part_spare_pose):
							print "before_hold_part : [WRONG_PART_POSE(part2or3) ==> CHANGE TO SPARE_POSE]"
							move_part_to_place_pose(CLASS_SET,target_part,Part_ulol.part_spare_pose)
					else:
						hold_point_offset = 1				
						
				elif hole_num is 5:
					if part_num == 2:
						hold_point_offset = 2
						if not Is_part_on_proper_pose(PART_SCENE.TF_List,target_part,Part_ulol.part_init_pose):
							print "before_hold_part : [WRONG_PART_POSE(part2or3) ==> CHANGE TO INIT_POSE]"
							move_part_to_place_pose(CLASS_SET,target_part,Part_ulol.part_init_pose)	
					else:
						hold_point_offset = 0
						
				else:
					hold_point_offset = None

			elif part_num == 3:
				hold_point_offset = None
				# move_part_to_place_pose(CLASS_SET,target_part,Part_ulol.part_init_pose,0)

			elif part_num in [4,5]:
				if hole_num in [0,1]:
					print "HOLE NUM 1,2"
					if not Is_part_on_proper_pose(PART_SCENE.TF_List,target_part,Part_ulol.part_init_pose)\
							or Is_part_on_proper_pose(PART_SCENE.TF_List,target_part,Part_ulol.part_ready_pose):
						print "before_hold_part : [WRONG_PART_POSE(part5or6) ==> CHANGE TO INIT_POSE]"
						move_part_to_place_pose(CLASS_SET,target_part,Part_ulol.part_init_pose)
					hold_point_offset = 4

				elif hole_num in [2,3]:
					print "HOLE NUM 3,4"
					if not Is_part_on_proper_pose(PART_SCENE.TF_List,target_part,Part_ulol.part_spare_pose):
						print "before_hold_part : [WRONG_PART_POSE(part5or6) ==> CHANGE TO SPARE_POSE]"
						move_part_to_place_pose(CLASS_SET,target_part,Part_ulol.part_spare_pose)
					hold_point_offset = 3


				elif hole_num in [4,5,6]:
					print "HOLE NUM 5,6,7"
					if not Is_part_on_proper_pose(PART_SCENE.TF_List,target_part,Part_ulol.part_spare_pose):
						print "before_hold_part : [WRONG_PART_POSE(part5or6)==> CHANGE TO SPARE_POSE]"
						move_part_to_place_pose(CLASS_SET,target_part,Part_ulol.part_spare_pose)
					hold_point_offset = 4

			print "HOLD_POSE_NUM :",hold_point_offset
			hold_part(CLASS_SET, target_part, hold_point_offset)		
		
			pin_num = Part_ulol.pin_name.index(target_pin)			

			grab_pin(CLASS_SET,target_pin,pin_tag)
			pin_num = Part_ulol.pin_name.index(target_pin)
			pin_length_offset = Part_ulol.pin_length[pin_num]+0.02
			hole_pose = deepcopy(PART_SCENE.TF_List[part_num]['holes'][hole_num])
			g_mat  = Part_ulol.mat_from_pose(hole_pose)
			gp_mat = numpy.dot(g_mat,MGW.tf_to_mat([0, 0, -pin_length_offset], [0, 0, 0]))
			target_pose = Part_ulol.pose_from_mat(gp_mat)
			
			put_the_part_down(CLASS_SET, 'rob1', target_pin_name, target_pose)
			PART_SCENE.assemble_pin(target_part,target_pin_name)
			print "ASM_List : ",PART_SCENE.ASM_List
			mat1 = Part_ulol.mat_from_pose(hole_pose)

			trans = numpy.array(Part_ulol.pin_TF_pose[pin_num]['trans'])+numpy.array([0,0,-(Part_ulol.pin_length[pin_num])])
			mat2 = Part_ulol.translation_matrix(trans)
			
			rpy  = Part_ulol.pin_TF_pose[pin_num]['rot']
			mat3 = Part_ulol.euler_matrix(-rpy[0],rpy[1],rpy[2])
			
			mat  = (mat1.dot(mat2)).dot(mat3)
			goal = Part_ulol.pose_from_mat(mat)
			PART_SCENE.change_pin_org(target_pin_name,goal)

	print "=========================================================[INSERT_PIN]\n"
def assemble_part(CLASS_SET, assmebling_part_name):
	print "\n[ASSEMBLE_PART]======================================================"

	PART_SCENE = CLASS_SET['tf']
	target_part = 'chair_part6'
	part_num = Part_ulol.part_name.index(assmebling_part_name)

	if assmebling_part_name == 'chair_part5':
		gp_num = 3
		hold_num = None
		offset_vector = 0.1
		assemble_pose_offset = assembly_data.chair_part5_assemble_offset
		CLASS_SET['rob1'].go([0,0,-1.57,0,0,0], wait=True)
	else:
		if not Is_part_on_proper_pose(PART_SCENE.TF_List,target_part,Part_ulol.part12_assembling_pose):
			print "before_hold_part : [WRONG_PART_POSE ==> CHANGE TO assembling_POSE]"
			move_part_to_place_pose(CLASS_SET,target_part,Part_ulol.part12_assembling_pose)
	
		if assmebling_part_name == 'chair_part2':
			gp_num = 1
			hold_num = 2
			offset_vector = [0, 0.1,0]
			assemble_pose_offset = assembly_data.chair_part2_assemble_offset

		elif assmebling_part_name == 'chair_part3':
			gp_num = 0
			hold_num = 0
			offset_vector = [0,-0.1,0]
			assemble_pose_offset = assembly_data.chair_part3_assemble_offset

		elif assmebling_part_name == 'chair_part4':
			gp_num = 1
			hold_num = 0
			offset_vector = [0,0.1,0]
			assemble_pose_offset = assembly_data.chair_part4_assemble_offset
			
		else:
			print "(assemble_part):WRONG_PART_NUM"
			return

	chair_part6_org = PART_SCENE.TF_List[5]['origin']
	complete_assemble_org = PART_SCENE.get_TF_pose(chair_part6_org,assemble_pose_offset)

	assemble_pose_offset['trans'][2] -= 0.020

	assemble_org = PART_SCENE.get_TF_pose(chair_part6_org,assemble_pose_offset)

	tf_offset = Part_ulol.GP.grasping_pose[part_num][gp_num]

	assemble_pose = PART_SCENE.get_TF_pose(assemble_org,tf_offset)

	complete_assemble_pose = PART_SCENE.get_TF_pose(complete_assemble_org,tf_offset)

	

	hold_part(CLASS_SET,target_part,hold_num,'rob1')
	pick_the_mesh_up(CLASS_SET,'rob2',assmebling_part_name,gp_num,True)
	put_the_part_down(CLASS_SET,'rob2',assmebling_part_name,assemble_pose,offset_vector)
	PART_SCENE.assemble_part(target_part,assmebling_part_name)

	PART_SCENE.change_part_org(part_num,complete_assemble_org)
	print "======================================================[ASSEMBLE_PART]\n"

def attaching_test():
	try:
		rospy.init_node('pick_and_place_test', anonymous=True)
		MG_ROB1 = MGW.MoveGroupCommanderWrapper('rob1_arm')
		MG_ROB2 = MGW.MoveGroupCommanderWrapper('rob2_arm')
		PART = Part_ulol.TF_Node()
		listener = tf.TransformListener()
		CLASS_SET = {'rob1':MG_ROB1,'rob2':MG_ROB2,'tf':PART,'listener':listener}
		

		PART.set_parts([1,5],[],Part_ulol.part12_assembling_pose)

		MG_ROB1.go_to_initial_pose()
		MG_ROB2.go_to_initial_pose()
		move_part_to_place_pose(CLASS_SET,'chair_part2', Part_ulol.part_ready_pose)
		move_part_to_place_pose(CLASS_SET,'chair_part3', Part_ulol.part_ready_pose)

		assemble_part(CLASS_SET,'chair_part3')
		assemble_part(CLASS_SET,'chair_part2')
		assemble_part(CLASS_SET,'chair_part4')
		print "ENTER"
		raw_input()
		move_part_to_place_pose(CLASS_SET,P6,Part_ulol.part_spare_pose,0,'rob1')
		print "ENTER"
		raw_input()
		assemble_part(CLASS_SET,'chair_part5')

		
	

	except Exception as e:
		print e
def insert_pin_test():
	try:
		rospy.init_node('pick_and_place_test', anonymous=True)
		MG_ROB1 = MGW.MoveGroupCommanderWrapper('rob1_arm')
		MG_ROB2 = MGW.MoveGroupCommanderWrapper('rob2_arm')
		PART = Part_ulol.TF_Node()
		listener = tf.TransformListener()
		CLASS_SET = {'rob1':MG_ROB1,'rob2':MG_ROB2,'tf':PART,'listener':listener}
		MG_ROB1.go_to_initial_pose()
		MG_ROB2.go_to_initial_pose()

		PART.set_parts([5])

		print "ENTER",
		raw_input()

		target_pin = Part_ulol.pin_name[0]
		target_part = Part_ulol.part_name[5]
		for i in range(10):
			if not i in [1,4,9]:
				insert_pin(CLASS_SET,target_pin,i,target_part,i)

		move_part_to_place_pose(CLASS_SET,'chair_part6')
		move_part_to_place_pose(CLASS_SET,'chair_part3')
		move_part_to_place_pose(CLASS_SET,'chair_part2')

		target_pin = Part_ulol.pin_name[2]
		target_part = Part_ulol.part_name[1]
		print target_pin
		print target_part

		for i in range(6):
			insert_pin(CLASS_SET,target_pin,i,target_part,i)

	except Exception as e:
		print e

def main():
	try:
		rospy.init_node('pick_and_place_test', anonymous=True)
		MG_ROB1 = MGW.MoveGroupCommanderWrapper('rob1_arm')
		MG_ROB2 = MGW.MoveGroupCommanderWrapper('rob2_arm')
		PART = Part_ulol.TF_Node()
		listener = tf.TransformListener()
		CLASS_SET = {'rob1':MG_ROB1,'rob2':MG_ROB2,'tf':PART,'listener':listener}
		
		PART.set_parts([0,1,2,3,4,5],[0,2])

		MG_ROB1.go_to_initial_pose()
		MG_ROB2.go_to_initial_pose()

		P2 = Part_ulol.part_name[1]
		P3 = Part_ulol.part_name[2]
		P4 = Part_ulol.part_name[3]
		P5 = Part_ulol.part_name[4]
		P6 = Part_ulol.part_name[5]

		p1 = Part_ulol.pin_name[0]
		p3 = Part_ulol.pin_name[2]


		for i in [0,1,2,3]:
			print i
			if i in [0,1]:
				insert_pin(CLASS_SET,p3,i,P2,i+4)
			else:
				insert_pin(CLASS_SET,p3,i,P3,i+4-2)

		move_part_to_place_pose(CLASS_SET,P2,Part_ulol.part_ready_pose)
		move_part_to_place_pose(CLASS_SET,P3,Part_ulol.part_ready_pose)

		pin = 0

		for i in range(7):
			insert_pin(CLASS_SET,p1,pin,P6,i)
			pin += 1

		assemble_part(CLASS_SET,'chair_part3')
		assemble_part(CLASS_SET,'chair_part2')
		assemble_part(CLASS_SET,'chair_part4')

		for hole in [2,3]:
			insert_pin(CLASS_SET,p1,pin,P2,hole)
			pin += 1

		MG_ROB2.go([0,0,-1.57,0,0,0], wait=True)
		move_part_to_place_pose(CLASS_SET,P6,Part_ulol.part_spare_pose,gp = 0,rob = 'rob1',holding = True)

		for hole in [0,1]:
			insert_pin(CLASS_SET,p1,pin,P3,hole)
			pin += 1
		
		for hole in [2,3,4]:		
			insert_pin(CLASS_SET,p1,pin,P4,hole)
			pin += 1

		assemble_part(CLASS_SET,'chair_part5')
		MG_ROB2.go([0,0,-1.57,0,0,0], wait=True)
		MG_ROB1.go([0,0,-1.57,0,0,0], wait=True)

	except Exception as e:
		print e

if __name__ == '__main__':
  main()