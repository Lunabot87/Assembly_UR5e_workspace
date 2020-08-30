from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from tf.transformations import quaternion_from_euler

part1_pose = PoseStamped()
part2_pose = PoseStamped()
part3_pose = PoseStamped()
part4_pose = PoseStamped()
part5_pose = PoseStamped()
part6_pose = PoseStamped()

part_ready_pose = [part1_pose,part2_pose,part3_pose,part4_pose,part5_pose,part6_pose]

for i in range(6):
	part_ready_pose[i].header.frame_id = "world"
	part_ready_pose[i].pose.orientation.w = 1.0
	part_ready_pose[i].pose.position.z = 0.83

part1_pose.pose.position.x = 0
part1_pose.pose.position.y = 0.45
 
part2_pose.pose.position.x = -0.4
part2_pose.pose.position.y = -0.5
orientation_list = quaternion_from_euler(0,0,3.14)
part2_pose.pose.orientation.x = orientation_list[0]
part2_pose.pose.orientation.y = orientation_list[1]
part2_pose.pose.orientation.z = orientation_list[2]
part2_pose.pose.orientation.w = orientation_list[3]

part3_pose.pose.position.x = -0.4
part3_pose.pose.position.y = -0.6
part3_pose.pose.orientation.x = orientation_list[0]
part3_pose.pose.orientation.y = orientation_list[1]
part3_pose.pose.orientation.z = orientation_list[2]
part3_pose.pose.orientation.w = orientation_list[3]

part4_pose.pose.position.x = -0.55
part4_pose.pose.position.y = -0.8
orientation_list = quaternion_from_euler(0,0,1.57)
part4_pose.pose.orientation.x = orientation_list[0]
part4_pose.pose.orientation.y = orientation_list[1]
part4_pose.pose.orientation.z = orientation_list[2]
part4_pose.pose.orientation.w = orientation_list[3]

part5_pose.pose.position.x = 0.4
part5_pose.pose.position.y = -1.0
orientation_list = quaternion_from_euler(3.14,-0.09,-1.57)
part5_pose.pose.orientation.x = orientation_list[0]
part5_pose.pose.orientation.y = orientation_list[1]
part5_pose.pose.orientation.z = orientation_list[2]
part5_pose.pose.orientation.w = orientation_list[3]

part6_pose.pose.position.x = 0.45
part6_pose.pose.position.y = 0.15
orientation_list = quaternion_from_euler(3.14,0.09,-1.57)
part6_pose.pose.orientation.x = orientation_list[0]
part6_pose.pose.orientation.y = orientation_list[1]
part6_pose.pose.orientation.z = orientation_list[2]
part6_pose.pose.orientation.w = orientation_list[3]