# joint, link names
ROB1_JOINTS = ['rob1_shoulder_pan_joint','rob1_shoulder_lift_joint','rob1_elbow_joint',
        'rob1_wrist_1_joint','rob1_wrist_2_joint','rob1_wrist_3_joint']

ROB2_JOINTS = ['rob2_shoulder_pan_joint','rob2_shoulder_lift_joint','rob2_elbow_joint',
        'rob2_wrist_1_joint','rob2_wrist_2_joint','rob2_wrist_3_joint'] 

ROB1_ARM_LINK = ['rob1_base_link', 'rob1_shoulder_link', 'rob1_upper_arm_link', 'rob1_forearm_link', 
        'rob1_wrist_1_link', 'rob1_wrist_2_link', 'rob1_wrist_3_link']

ROB1_GRIPPER_LINK = ['rob1_robotiq_arg2f_base_link', 
		'rob1_left_outer_knuckle', 'rob1_left_outer_finger', 
		'rob1_left_inner_finger', 'rob1_left_inner_finger_pad', 'rob1_left_inner_knuckle',
		'rob1_right_outer_knuckle', 'rob1_right_outer_finger', 
		'rob1_right_inner_finger', 'rob1_right_inner_finger_pad', 'rob1_right_inner_knuckle']

ROB2_ARM_LINK = ['rob2_base_link', 'rob2_shoulder_link', 'rob2_upper_arm_link', 'rob2_forearm_link', 
        'rob2_wrist_1_link', 'rob2_wrist_2_link', 'rob2_wrist_3_link']

ROB2_GRIPPER_LINK = ['rob2_robotiq_arg2f_base_link', 
		'rob2_left_outer_knuckle', 'rob2_left_outer_finger', 
		'rob2_left_inner_finger', 'rob2_left_inner_finger_pad', 'rob2_left_inner_knuckle',
		'rob2_right_outer_knuckle', 'rob2_right_outer_finger', 
		'rob2_right_inner_finger', 'rob2_right_inner_finger_pad', 'rob2_right_inner_knuckle']



# part paths
PART_DIR = "../../assembly_robot_description/stl/stefan_chair_part/"
PART1_PATH = PART_DIR + "chair_part1.SLDPRT.STL"
PART2_PATH = PART_DIR + "chair_part2.SLDPRT.STL"
PART3_PATH = PART_DIR + "chair_part3.SLDPRT.STL"
PART4_PATH = PART_DIR + "chair_part4.SLDPRT.STL"
PART5_PATH = PART_DIR + "chair_part5.SLDPRT.STL"
PART6_PATH = PART_DIR + "chair_part6.SLDPRT.STL"

# actual grasp pose wrt real_ee_link
TCP_OFFSET = {'real_ee_link': 0.0, 'tcp_gripper_open' : 0.13, 'tcp_gripper_closed' : 0.14}