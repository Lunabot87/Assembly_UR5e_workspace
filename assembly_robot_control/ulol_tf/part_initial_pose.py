from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler

part1_pose = PoseStamped()
part2_pose = PoseStamped()
part3_pose = PoseStamped()
part4_pose = PoseStamped()
part5_pose = PoseStamped()
part6_pose = PoseStamped()

pin101_pose = PoseStamped()
pin104_pose = PoseStamped()
pin126_pose = PoseStamped()
pin129_pose = PoseStamped()

part_pose = [part1_pose,part2_pose,part3_pose,part4_pose,part5_pose,part6_pose]
pin_pose = [pin101_pose,pin104_pose,pin126_pose,pin129_pose]

for i in range(6):
	part_pose[i].header.frame_id = "world"
	part_pose[i].pose.orientation.w = 1.0
	part_pose[i].pose.position.z = 0.83
	if i < 4:
		pin_pose[i].header.frame_id = "world"
		pin_pose[i].pose.orientation.w = 1.0
		pin_pose[i].pose.position.z = 0.83

part1_pose.pose.position.x = 0.3
part1_pose.pose.position.y = 0.5
 
part2_pose.pose.position.x = 0.2
part2_pose.pose.position.y = -0.35

part3_pose.pose.position.x = -0.2
part3_pose.pose.position.y = 0.35

part4_pose.pose.position.x = -0.41
part4_pose.pose.position.y = -0.47

part5_pose.pose.position.x = -0.45
part5_pose.pose.position.y = 0.15
orientation_list = quaternion_from_euler(0,0.09,-1.57)
part5_pose.pose.orientation.x = orientation_list[0]
part5_pose.pose.orientation.y = orientation_list[1]
part5_pose.pose.orientation.z = orientation_list[2]
part5_pose.pose.orientation.w = orientation_list[3]

part6_pose.pose.position.x = 0.45
part6_pose.pose.position.y = -0.15
orientation_list = quaternion_from_euler(0,-0.09,1.57)
part6_pose.pose.orientation.x = orientation_list[0]
part6_pose.pose.orientation.y = orientation_list[1]
part6_pose.pose.orientation.z = orientation_list[2]
part6_pose.pose.orientation.w = orientation_list[3]

pin101_pose.pose.position.x = -0.6
pin101_pose.pose.position.y = 0.5

pin104_pose.pose.position.x = -0.6
pin104_pose.pose.position.y = 0.4

pin126_pose.pose.position.x = -0.6
pin126_pose.pose.position.y = 0.3

pin129_pose.pose.position.x = -0.6
pin129_pose.pose.position.y = 0.2


