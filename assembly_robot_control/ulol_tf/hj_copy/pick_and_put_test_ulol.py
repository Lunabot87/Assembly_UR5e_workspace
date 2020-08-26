import rospy
import tf
import numpy
import random
import Part_ulol
from math import pi
from copy import deepcopy
import move_group_wrapper_0801 as MGW

def add_mesh_at_random_pose(CLASS_SET,mesh_name):
	PART_SCENE = CLASS_SET['tf']

	if not mesh_name in Part_ulol.part_name:
		print '[WRONG PART NAME]'
	else:
		pose = Part_ulol.PoseStamped()
		trans = [random.uniform(-0.4,0.4),random.uniform(-0.5,0.5),0.81]
		if mesh_name == 'chair_part5':
			rot = [0,0.09,random.uniform(-pi,pi)]
		elif mesh_name == 'chair_part6':
			rot = [3.14,0.09,random.uniform(-pi,pi)]
			trans[2] = 0.825
		else:
			rot = [0,0,random.uniform(-pi,pi)]
		pose.pose = Part_ulol.pose_from_transrot(trans,rot)

		mesh_num = Part_ulol.part_name.index(mesh_name)
		mesh_file = Part_ulol.part_file[mesh_num]
		PART_SCENE.add_mesh(mesh_name,mesh_file,pose)	
def pose_compare(target_pose,cur_pose):
	target_pose_quat = Part_ulol.make_orientation_list(target_pose.orientation)
	target_pose_trans = Part_ulol.make_position_list(target_pose.position)
	target_pose_elements = target_pose_trans + target_pose_quat

	cur_pose_quat = Part_ulol.make_orientation_list(cur_pose.orientation)
	cur_pose_trans = Part_ulol.make_position_list(cur_pose.position)
	cur_pose_elements = cur_pose_trans + cur_pose_quat

	target_pose_round = [round(x,4) for x in target_pose_elements]
	cur_pose_round = [round(x,4) for x in cur_pose_elements]

	if target_pose_round == cur_pose_round:
		return True
	else:
		return False
def Is_part_on_proper_pose(part_tf_list, target_part_name):
	print target_part_name
	part_num = Part_ulol.part_name.index(target_part_name)
	cur_pose = part_tf_list[part_num]['origin']
	proper_pose = Part_ulol.part_pose[part_num].pose
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
def which_rob_is_near_to(gp_pose, part_name = None):
	if part_name == 'chair_part6':
		criteria = 0.35
	elif part_name =='chair_part5':
		criteria = -0.25
	else:
		criteria = 0.05
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


def pick_the_mesh_up(CLASS_SET, rob_name, target_mesh_name, tag_offset):
	
	ROB = CLASS_SET[rob_name]
	if rob_name == 'rob1':
		rob_base = 'rob1_real_base_link'
	elif rob_name == 'rob2':
		rob_base = 'rob2_real_base_link'
	PART_SCENE = CLASS_SET['tf']

	if target_mesh_name in Part_ulol.part_name:
		grasp_point = target_mesh_name+'-GRASP-'+str(tag_offset+1)

	elif target_mesh_name in Part_ulol.pin_name:
		grasp_point = target_mesh_name+'-'+str(tag_offset+1)
		target_pin_name = target_mesh_name+'-'+str(tag_offset+1)

	else:
		print "WRONG NAME"
		return

	print "ENTER",
	raw_input()

	(g_trans, g_rot) \
     = CLASS_SET['listener'].lookupTransform(rob_base, grasp_point, rospy.Time(0))
	offset = 0.2
	(pg_trans, pg_rot) = ROB._grasp_to_pregrasp(g_trans, g_rot, offset)
	(result0, _) = ROB._get_best_ik_plan(g_trans, g_rot)
	ROB.go_to_pose_goal(pg_trans, pg_rot, result0)
	ROB.go_linear_to_pose_goal(g_trans, g_rot, result0)

	attach_list = PART_SCENE.get_attach_list(target_pin_name)
	print "ATTACH = ", attach_list
	attach(CLASS_SET,rob_name,attach_list)
	ROB.go_linear_to_pose_goal(pg_trans, pg_rot, result0)

def put_the_part_down(CLASS_SET, rob_name, target_part_name, goal_pose):
	ROB = CLASS_SET[rob_name]
	if rob_name == 'rob1':
		rob_base = 'rob1_real_base_link'
	elif rob_name == 'rob2':
		rob_base = 'rob2_real_base_link'
	PART_SCENE = CLASS_SET['tf']

	goal_pose_name = 'GOAL_POSE'
	PART_SCENE.send_temporary_TF(goal_pose, goal_pose_name)

	print "GOAL_POSE_SENT",
	raw_input()

	(g_trans, g_rot) \
    = CLASS_SET['listener'].lookupTransform(rob_base, goal_pose_name, rospy.Time(0))

	offset = 0.2
	
	(pg_trans, pg_rot) = ROB._grasp_to_pregrasp(g_trans, g_rot, offset)
	(result0, _) = ROB._get_best_ik_plan(g_trans, g_rot)
	ROB.go_to_pose_goal(pg_trans, pg_rot, result0)
	ROB.go_linear_to_pose_goal(g_trans, g_rot, result0)
	print target_part_name
	attach_list = PART_SCENE.get_attach_list(target_part_name)
	detach(CLASS_SET,rob_name,attach_list)
	PART_SCENE.init_attach_list()
	ROB.go_linear_to_pose_goal(pg_trans, pg_rot, result0)
	ROB.go_to_initial_pose()

def move_part_to_place_pose(CLASS_SET,target_part_name):
	PART_SCENE = CLASS_SET['tf']
	if Is_part_on_proper_pose(PART_SCENE.TF_List,target_part_name):
		print "Already good pose"
	else:
		part_num = Part_ulol.part_name.index(target_part_name)
		num_of_gp = Part_ulol.grasp_in_part[part_num]
		gp_list = PART_SCENE.GP_List[part_num]['pose']
		dist_to_table_org=[]
		
		# the nearist GP select
		for i in range(num_of_gp):
			gp_position = gp_list[i].position
			gp_point = numpy.array([gp_position.x,gp_position.y-0.05,gp_position.z])
			dist = numpy.linalg.norm(gp_point)
			dist_to_table_org.append(deepcopy(dist))

		cur_gp_num = dist_to_table_org.index(min(dist_to_table_org))
		cur_gp_pose = gp_list[cur_gp_num]

		# proper selected gp pose of proper org pose
		proper_org = Part_ulol.part_pose[part_num].pose
		tf_offset = Part_ulol.GP.grasping_pose[part_num][cur_gp_num]
		proper_gp_pose = PART_SCENE.get_TF_pose(proper_org, tf_offset)

		rob_near_to_cur = which_rob_is_near_to(cur_gp_pose, target_part_name)
		rob_near_to_prop = which_rob_is_near_to(proper_gp_pose, target_part_name)

		if not rob_near_to_cur == which_rob_is_near_to(cur_gp_pose):
				if rob_near_to_cur == 'rob1':
					rob_near_to_cur = 'rob2'
				elif rob_near_to_cur == 'rob2':
					rob_near_to_cur = 'rob1'

		pick_the_mesh_up(CLASS_SET, rob_near_to_cur, target_part_name, cur_gp_num)

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
			move_part_to_place_pose(CLASS_SET,target_part_name)

def grab_pin(CLASS_SET, pin_name,pin_tag):
	
	pick_the_mesh_up(CLASS_SET,'rob1',pin_name,pin_tag)

def insert_spiral_motion(CLASS_SET, target_pin, pin_tag, target_part, hole_num):
# pin_name = ex)pin101340-2, # hole_num starts with 1
	if not target_part in Part_ulol.part_name:
		print "[insert_spiral_motion] : WRONG_TARGET_PART"
		return
	else:
		PART_SCENE = CLASS_SET['tf']
		part_num = Part_ulol.part_name.index(target_part)
		if not hole_num-1 in range(Part_ulol.holes_in_part[part_num]):
			print "[insert_spiral_motion] : WRONG_HOLE_NUMBER"
		else:
			grab_pin(CLASS_SET,target_pin,pin_tag)
			print "ENTER",
			raw_input()
			pin_num = Part_ulol.pin_name.index(target_pin)
			pin_length_offset = Part_ulol.pin_length[pin_num] + 0.02

			goal_pose = deepcopy(PART_SCENE.TF_List[part_num]['holes'][hole_num-1])
			
			g_mat  = Part_ulol.mat_from_pose(goal_pose)
			print g_mat
			gp_mat = numpy.dot(g_mat,MGW.tf_to_mat([0, 0, -pin_length_offset], [0, 0, 0]))
			target_pose = Part_ulol.pose_from_mat(gp_mat)
			print gp_mat
			target_pin_name = target_pin+'-'+str(pin_tag+1)
			put_the_part_down(CLASS_SET, 'rob1', target_pin_name, target_pose)
			PART_SCENE.assemble_pin(target_part,target_pin_name)
			print "ASM_List : ",PART_SCENE.ASM_List



def pick_and_put_test():
	print "pick_and_put_part_test"
	try:
		rospy.init_node('pick_and_place_test', anonymous=True)
		MG_ROB1 = MGW.MoveGroupCommanderWrapper('rob1_arm')
		MG_ROB2 = MGW.MoveGroupCommanderWrapper('rob2_arm')
		PART = Part_ulol.TF_Node()
		listener = tf.TransformListener()
		CLASS_SET = {'rob1':MG_ROB1,'rob2':MG_ROB2,'tf':PART,'listener':listener}
		target_q = MG_ROB1.get_named_target_values('rob1_pin1_camera_check')
		MG_ROB1.go(target_q)
		MG_ROB2.go_to_initial_pose()
		PART.set_parts([],[0])
		print "ENTER",
		raw_input()
		add_mesh_at_random_pose('chair_part3')
		print "ENTER",
		raw_input()
		pick_the_mesh_up(CLASS_SET,'rob2','chair_part3',1)

		print "ENTER",
		raw_input()		
		goal_pose = Part_ulol.PoseStamped().pose
		goal_pose.orientation.x = 0.707
		goal_pose.orientation.y = 0.707
		goal_pose.position.z = 0.81
		put_the_part_down(CLASS_SET,'rob2','chair_part3',goal_pose)

	except Exception as e:
		print e

	print "ENTER"
	raw_input()
def move_part_to_place_pose_test():
	try:
		rospy.init_node('pick_and_place_test', anonymous=True)
		MG_ROB1 = MGW.MoveGroupCommanderWrapper('rob1_arm')
		MG_ROB2 = MGW.MoveGroupCommanderWrapper('rob2_arm')
		PART = Part_ulol.TF_Node()
		listener = tf.TransformListener()
		CLASS_SET = {'rob1':MG_ROB1,'rob2':MG_ROB2,'tf':PART,'listener':listener}
		target_q = MG_ROB1.get_named_target_values('rob1_pin1_camera_check')
		MG_ROB1.go(target_q)
		MG_ROB2.go_to_initial_pose()

		PART.set_parts([],[])

		print "ENTER",
		raw_input()
		add_mesh_at_random_pose(CLASS_SET,'chair_part2')
		print "ENTER",
		raw_input()
		add_mesh_at_random_pose(CLASS_SET,'chair_part3')
		print "ENTER",
		raw_input()
		add_mesh_at_random_pose(CLASS_SET,'chair_part6')
		print "ENTER",
		raw_input()
		move_part_to_place_pose(CLASS_SET,'chair_part6')
		move_part_to_place_pose(CLASS_SET,'chair_part3')
		move_part_to_place_pose(CLASS_SET,'chair_part2')
	

	except Exception as e:
		print e
def grab_pin_test():
	try:
		rospy.init_node('pick_and_place_test', anonymous=True)
		MG_ROB1 = MGW.MoveGroupCommanderWrapper('rob1_arm')
		MG_ROB2 = MGW.MoveGroupCommanderWrapper('rob2_arm')
		PART = Part_ulol.TF_Node()
		listener = tf.TransformListener()
		CLASS_SET = {'rob1':MG_ROB1,'rob2':MG_ROB2,'tf':PART,'listener':listener}
		target_q = MG_ROB1.get_named_target_values('rob1_pin1_camera_check')
		MG_ROB1.go(target_q)
		MG_ROB2.go_to_initial_pose()

		PART.set_parts([])

		target_pin = Part_ulol.pin_name[0]
		grab_pin(CLASS_SET,target_pin,0)
	

	except Exception as e:
		print e
def insert_spiral_motion_test():
	try:
		rospy.init_node('pick_and_place_test', anonymous=True)
		MG_ROB1 = MGW.MoveGroupCommanderWrapper('rob1_arm')
		MG_ROB2 = MGW.MoveGroupCommanderWrapper('rob2_arm')
		PART = Part_ulol.TF_Node()
		listener = tf.TransformListener()
		CLASS_SET = {'rob1':MG_ROB1,'rob2':MG_ROB2,'tf':PART,'listener':listener}
		MG_ROB1.go_to_initial_pose()
		MG_ROB2.go_to_initial_pose()

		PART.set_parts([5],[0])

		target_pin = Part_ulol.pin_name[0]
		target_part = Part_ulol.part_name[5]
		insert_spiral_motion(CLASS_SET,target_pin,0,target_part,1)
	

	except Exception as e:
		print e



	

if __name__ == '__main__':
  insert_spiral_motion_test()