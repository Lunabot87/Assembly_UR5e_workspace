import rospy
import random
import moveit_commander
import geometry_msgs
from tf import *
from moveit_commander.conversions import list_to_pose,pose_to_list

from assembly_robot_control.msg import const_pose
from assembly_robot_control.msg import for_tf_msg as tf_msg
from assembly_robot_control.msg import for_tf_msg_list as tf_msg_list

import Part_Pin.part_info as part_info					# part_file_address, part_name
from Part_Pin.pin_base import pin_TF_pose

import Part_Pin.hole_offsets as HO 		# locations of holes of each part


def get_const_pose(frame_id = 'part5_1', ed_trans= [0,0,0], ed_rpy=[0,0,0], et_trans= [0,0,0], et_rpy=[0,0,0], hole_num = 1):
	c_pose = const_pose()
	
	c_pose.const_name = frame_id+'-hole_'+str(hole_num)

	c_pose.end_posestamped.header.stamp.nsecs = 27000000
	c_pose.end_posestamped.header.frame_id = frame_id
	c_pose.end_posestamped.pose = list_to_pose(ed_trans+ed_rpy)

	c_pose.entry_posestamped.header.stamp.nsecs = 27000000
	c_pose.entry_posestamped.header.frame_id = frame_id
	c_pose.entry_posestamped.pose = list_to_pose(et_trans+et_rpy)

	return c_pose


def publish_tf_msg_list(tf_msg_list):
	br = TransformBroadcaster()
	scene = moveit_commander.PlanningSceneInterface()

	list_len = len(tf_msg_list)
	
	for i in range(list_len):
		target_part = tf_msg_list[i]
		mesh_name = 'chair_'+ target_part.part_name.split('_')[0]

		if not mesh_name in part_info.part_name:
			print "(publish_tf_msg_list) : WRONG part_name"
			return
		
		else:
			# ADD_MESH(..............)
			pose = tf_msg_list[i].part_pose
			file_name = part_info.part_file[part_info.part_name.index(mesh_name)]
			print "ENTER",
			raw_input()
			scene.add_mesh(mesh_name, pose, file_name, size=(1, 1, 1))

			pose_list = pose_to_list(tf_msg_list[i].part_pose.pose)
			br.sendTransform(pose_list[:3], pose_list[3:], rospy.Time.now()
													,target_part.part_name, target_part.part_pose.header.frame_id)
			print "ENTER",
			raw_input()
			const_list_len = len(target_part.const_pose_list)
		 
			for j in range(const_list_len):
				name = target_part.const_pose_list[j].const_name

				end = target_part.const_pose_list[j].end_posestamped
				end_pose = end.pose
				ed_pose_list = pose_to_list(end_pose)
				br.sendTransform(ed_pose_list[:3],ed_pose_list[3:],rospy.Time.now(),name+'end',end.header.frame_id)

				entry = target_part.const_pose_list[j].entry_posestamped
				entry_pose = entry.pose
				end_pose = end.pose
				et_pose_list = pose_to_list(entry_pose)
				br.sendTransform(et_pose_list[:3],et_pose_list[3:],rospy.Time.now(),name+'entry',entry.header.frame_id)

def main():
	try:
		rospy.init_node('tf_msg_test', anonymous=True)

		TF5 = tf_msg()
		TF5.part_name = 'part5_1'
		TF5.part_pose.header.frame_id = 'world'
		# TF.part_pose.pose = list_to_pose([0,0,0.84,3.14,0.09,-1.57])
		# CP1 = get_const_pose('part5_1',[0.4290,0.8602,-0.042],[0,0,0,1],[0.4290,0.8602,-0.05],[0,0,0,1],10)
		# CP2 = get_const_pose('part5_1',[0.3932,0.5673,-0.042],[0,0,0,1],[0.3932,0.5673,-0.05],[0,0,0,1],5)
		# CP3 = get_const_pose('part5_1',[0.0005,0.3947,-0.0063],[0,0,0,1],[0.0005,0.3947,-0.0143],[0,0,0,1],2)

		TF5.part_pose.pose = list_to_pose([-0.45,-0.4,0.85,0,0.09,0])
		CP501 = get_const_pose('part5_1',[0.002166, 0.426660, 0.005], [0,-3.14,0], [0.002166, 0.426660, 0.015], [0,-3.14,0],1)
		CP502 = get_const_pose('part5_1',[0.000492, 0.394704, 0.005], [0,-3.14,0], [0.000492, 0.394704, 0.015], [0,-3.14,0],2)
		CP503 = get_const_pose('part5_1',[0.369042, 0.407433, 0.040], [0,-3.14,0], [0.369042, 0.407433, 0.050], [0,-3.14,0],3)
		CP504 = get_const_pose('part5_1',[0.367368, 0.375477, 0.040], [0,-3.14,0], [0.367368, 0.375477, 0.050], [0,-3.14,0],4)
		CP505 = get_const_pose('part5_1',[0.393244, 0.567391, 0.035], [0,-3.14,0], [0.393244, 0.567391, 0.050], [0,-3.14,0],5)
		CP506 = get_const_pose('part5_1',[0.427122, 0.844371, 0.035], [0,-3.14,0], [0.427122, 0.844371, 0.050], [0,-3.14,0],6)
		CP507 = get_const_pose('part5_1',[0.431575, 0.876060, 0.035], [0,-3.14,0], [0.431578, 0.876060, 0.050], [0,-3.14,0],7)
		CP508 = get_const_pose('part5_1',[0.001329, 0.410682,-0.015], [0, 0, 0], [0.001329, 0.410682, 0.015], [0, 0,0],8)
		CP509 = get_const_pose('part5_1',[0.369205, 0.391455, 0.020], [0, 0, 0], [0.369205, 0.391455, 0.050], [0, 0,0],9)
		CP510 = get_const_pose('part5_1',[0.429349, 0.860216, 0.020], [0, 0, 0], [0.429349, 0.860216, 0.050], [0, 0,0],10)

		TF5.const_pose_list = [CP501,CP502,CP503,CP504,CP505,CP506,CP507,CP508,CP509,CP510]

		TF6 = tf_msg()
		TF6.part_name = 'part6_1'
		TF6.part_pose.header.frame_id = 'world'
		# TF.part_pose.pose = list_to_pose([0,0,0.84,3.14,0.09,-1.57])
		# CP1 = get_const_pose('part5_1',[0.4290,0.8602,-0.042],[0,0,0,1],[0.4290,0.8602,-0.05],[0,0,0,1],10)
		# CP2 = get_const_pose('part5_1',[0.3932,0.5673,-0.042],[0,0,0,1],[0.3932,0.5673,-0.05],[0,0,0,1],5)
		# CP3 = get_const_pose('part5_1',[0.0005,0.3947,-0.0063],[0,0,0,1],[0.0005,0.3947,-0.0143],[0,0,0,1],2)

		TF6.part_pose.pose = list_to_pose([0.05,-0.4,0.85,0,-0.09,0])
		CP601 = get_const_pose('part6_1',[0.002166, 0.426660,-0.015], [0,0,0], [0.002166, 0.426660,-0.005], [0, 0, 0], 1)
		CP602 = get_const_pose('part6_1',[0.000492, 0.394704,-0.015], [0,0,0], [0.000492, 0.394704,-0.005], [0, 0, 0], 2)
		CP603 = get_const_pose('part6_1',[0.369042, 0.407433,-0.050], [0,0,0], [0.369042, 0.407433,-0.040], [0, 0, 0], 3)
		CP604 = get_const_pose('part6_1',[0.367368, 0.375477,-0.050], [0,0,0], [0.367368, 0.375477,-0.040], [0, 0, 0], 4)
		CP605 = get_const_pose('part6_1',[0.393244, 0.567391,-0.050], [0,0,0], [0.393244, 0.567391,-0.035], [0, 0, 0], 5)
		CP606 = get_const_pose('part6_1',[0.427122, 0.844371,-0.050], [0,0,0], [0.427122, 0.844371,-0.035], [0, 0, 0], 6)
		CP607 = get_const_pose('part6_1',[0.431575, 0.876060,-0.050], [0,0,0], [0.431578, 0.876060,-0.035], [0, 0, 0], 7)
		CP608 = get_const_pose('part6_1',[0.001329, 0.410682,-0.015], [0, 3.14,0], [0.001329, 0.410682, 0.015], [0, 3.14, 0], 8)
		CP609 = get_const_pose('part6_1',[0.369205, 0.391455,-0.050], [0, 3.14,0], [0.369205, 0.391455,-0.020], [0, 3.14, 0], 9)
		CP610 = get_const_pose('part6_1',[0.429349, 0.860216,-0.050], [0, 3.14,0], [0.429349, 0.860216,-0.020], [0, 3.14, 0], 10)

		TF6.const_pose_list = [CP601,CP602,CP603,CP604,CP605,CP606,CP607,CP608,CP609,CP610]
	
		print "ENTER",
		raw_input()
		publish_tf_msg_list([TF6,TF5])




	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()