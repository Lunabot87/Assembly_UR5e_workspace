import ulol_tf_0727 as TF
import UR5_ulol_0728 as UR5

# TF.part_file TF.pin_file TF.part_name TF.pin_name
# TF.part_pose TF.pin_pose
# TF.HO.hole_offset

UR5.rospy.init_node('test0728', anonymous=True)

def get_target_hole(tf_node, part_number, hole_number):
	# part_number and hole_numbers are not their index
	part_number -= 1
	hole_number -= 1
	if part_number in range(6):
		
		nums_of_holes = len(TF.HO.hole_offset[part_number])
		if hole_number in range(nums_of_holes):
			target_pose = tf_node.TFlist[part_number]['holes'][hole_number]
		else:
			"WRONG HOLE NUMBER"
	else:
		print "WRONG PART NUMBER"

	return target_pose

def get_target_origin(tf_node,part_number):
	# part_number is not its index
	part_number -= 1
	if part_number in range(6):
		target_pose = tf_node.TFlist[part_number]['origin']
	else:
		print "WRONG PART NUMBER"
	return target_pose


def main():
	tf = TF.TF_Node()
	R1 = UR5.UR5()

	R1.go_to_initial_pose()
	tf.set_parts()

	print "ENTER",
	raw_input()

	tf.update_tf()

	print TF.part_name[2]
	target_pose = get_target_origin(tf,3)
	temp_orientation = R1.get_target_orientation(target_pose.orientation)
	target_pose.orientation = temp_orientation

	print target_pose

	print "ENTER",
	raw_input()

	R1.go_to_pose_goal(target_pose,0.4)

	print "ENTER",
	raw_input()
	R1.go_to_pose_goal(target_pose,0.3)
	R1.go_to_pose_goal(target_pose,0.1)

	R1.attach_part(TF.part_name[2],TF.part_file[2])

	print "ENTER",
	raw_input()

	R1.go_to_pose_goal(target_pose,0.4)

	print "ENTER",
	raw_input()

	R1.rotate_wrist([0.2,0.2,0.2])

	

	print "ENTER",
	raw_input()
	
	R1.go_to_pose_goal(target_pose,0.3)	

if __name__ == '__main__':
	main()
