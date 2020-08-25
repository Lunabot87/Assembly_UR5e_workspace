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


def get_const_pose(frame_id = 'part6_1', ed_trans= [0,0,0], ed_rpy=[0,0,0], et_trans= [0,0,0], et_rpy=[0,0,0], hole_num = 1):
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

		TF = tf_msg()
		TF.part_name = 'part6_1'
		TF.part_pose.header.frame_id = 'world'
		# TF.part_pose.pose = list_to_pose([0,0,0.84,3.14,0.09,-1.57])
		# CP1 = get_const_pose('part6_1',[0.4290,0.8602,-0.042],[0,0,0,1],[0.4290,0.8602,-0.05],[0,0,0,1],10)
		# CP2 = get_const_pose('part6_1',[0.3932,0.5673,-0.042],[0,0,0,1],[0.3932,0.5673,-0.05],[0,0,0,1],5)
		# CP3 = get_const_pose('part6_1',[0.0005,0.3947,-0.0063],[0,0,0,1],[0.0005,0.3947,-0.0143],[0,0,0,1],2)

		TF.part_pose.pose = list_to_pose([0,-0.005,0.886356,0,-0.09,0])
		CP01 = get_const_pose('part6_1',[0.002166,-0.426660, 0.020], [0,0,0,1], [0.002166,-0.426660, 0.030], [0,0,0,1],1)
		CP02 = get_const_pose('part6_1',[0.000492,-0.394704, 0.020], [0,0,0,1], [0.000492,-0.394704, 0.030], [0,0,0,1],2)
		CP03 = get_const_pose('part6_1',[0.369042,-0.407433, 0.055], [0,0,0,1], [0.369042,-0.407433, 0.065], [0,0,0,1],3)
		CP04 = get_const_pose('part6_1',[0.367368,-0.375477, 0.055], [0,0,0,1], [0.367368,-0.375477, 0.065], [0,0,0,1],4)
		CP05 = get_const_pose('part6_1',[0.393244,-0.567391, 0.000], [0,0,0,1], [0.393244,-0.567391, 0.065], [0,0,0,1],5)
		CP06 = get_const_pose('part6_1',[0.427122,-0.844371, 0.050], [0,0,0,1], [0.427122,-0.844371, 0.065], [0,0,0,1],6)
		CP07 = get_const_pose('part6_1',[0.431575,-0.876060, 0.050], [0,0,0,1], [0.431578,-0.876060, 0.065], [0,0,0,1],7)
		CP08 = get_const_pose('part6_1',[0.001329,-0.410682, 0.030], [0,0,0,1], [0.001329,-0.410682, 0.000], [0,0,0,1],8)
		CP09 = get_const_pose('part6_1',[0.369205,-0.391455, 0.065], [0,0,0,1], [0.369205,-0.391455, 0.035], [0,0,0,1],9)
		CP10 = get_const_pose('part6_1',[0.429349,-0.860216, 0.065], [0,0,0,1], [0.429349,-0.860216, 0.035], [0,0,0,1],10)

		TF.const_pose_list = [CP01,CP02,CP03,CP04,CP05,CP06,CP07,CP08,CP09,CP10]
		print TF
		print "ENTER",
		raw_input()
		publish_tf_msg_list([TF])




	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()