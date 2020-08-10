import rospy
import ulol_tf_0730 as PART
import ulol_UR5 as UR5
from copy import deepcopy
from math import pi




def point_from_pose(pose):
	point = []
	point.append(deepcopy(pose.position.x))
	point.append(deepcopy(pose.position.y))
	point.append(deepcopy(pose.position.z))
	return point


def main():	
	rospy.init_node('spiral_test', anonymous=True)
	try:
		TF = PART.TF_Node()
		R1 = UR5.UR5()
		R2 = UR5.UR5('rob2_arm')

		R1.go_to_joint_goal([pi/2,-pi/2,pi/2,-pi/2,-pi/2,pi])
		R2.go_to_joint_goal([pi/2,-pi/2,pi/2,0,0,0])
		TF.set_parts([1,2])

		target_part_num = 2
		hole_num = 7
		target_hole_pose = TF.return_part_hole(target_part_num,hole_num)
		target_hole_pose_base = R2.change_pose_cordinate(target_hole_pose
															,'world'
															,R2.real_base_name) 
		hole_point = point_from_pose(target_hole_pose_base)

		o_list = UR5.make_orientation_list(target_hole_pose_base.orientation)
		Matrix = UR5.quaternion_matrix(o_list)[:3,:3]
		axis = Matrix.dot([0,0,1])
		spiral = UR5.make_spiral_cmd(hole_point)

		R2.move_tool_trace(spiral,'-y',axis)
		raw_input()
		R2.go_to_joint_goal([pi/2,-pi/2,pi/2,0,0,0])

		target_part_num = 1
		hole_num = 7
		target_hole_pose = TF.return_part_hole(target_part_num,hole_num)
		target_hole_pose_base = R1.change_pose_cordinate(target_hole_pose
															,'world'
															,R1.real_base_name) 
		hole_point = point_from_pose(target_hole_pose_base)

		o_list = UR5.make_orientation_list(target_hole_pose_base.orientation)
		Matrix = UR5.quaternion_matrix(o_list)[:3,:3]
		axis = Matrix.dot([0,0,1])
		spiral = UR5.make_spiral_cmd(hole_point)

		R1.move_tool_trace(spiral,'z',axis)
		raw_input()
		R1.go_to_joint_goal([pi/2,-pi/2,pi/2,-pi/2,-pi/2,pi])

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == '__main__':
	main()