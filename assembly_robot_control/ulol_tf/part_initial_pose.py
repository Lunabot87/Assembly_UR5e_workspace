from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from pin_base import*

part1_pose = PoseStamped()
part2_pose = PoseStamped()
part3_pose = PoseStamped()
part4_pose = PoseStamped()
part5_pose = PoseStamped()
part6_pose = PoseStamped()

part_pose = [part1_pose,part2_pose,part3_pose,part4_pose,part5_pose,part6_pose]

for i in range(6):
	part_pose[i].header.frame_id = "world"
	part_pose[i].pose.orientation.w = 1.0
	part_pose[i].pose.position.z = 0.83

part1_pose.pose.position.x = 0.3
part1_pose.pose.position.y = 0.5
 
part2_pose.pose.position.x = -0.2
part2_pose.pose.position.y = 0.1
orientation_list = quaternion_from_euler(0,0,3.14)
part2_pose.pose.orientation.x = orientation_list[0]
part2_pose.pose.orientation.y = orientation_list[1]
part2_pose.pose.orientation.z = orientation_list[2]
part2_pose.pose.orientation.w = orientation_list[3]

part3_pose.pose.position.x = -0.2
part3_pose.pose.position.y = -0.1
orientation_list = quaternion_from_euler(0,0,3.14)
part3_pose.pose.orientation.x = orientation_list[0]
part3_pose.pose.orientation.y = orientation_list[1]
part3_pose.pose.orientation.z = orientation_list[2]
part3_pose.pose.orientation.w = orientation_list[3]

part4_pose.pose.position.x = -0.41
part4_pose.pose.position.y = -0.47

part5_pose.pose.position.x = -0.45
part5_pose.pose.position.y = 0.151
orientation_list = quaternion_from_euler(0,0.09,-1.57)
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


pin_arrayment = []

_temp_pin_pose = deepcopy(base11)
pin11_arrayment	= []
for i in range(3):
	_temp_pin_pose.pose.position.y = base11.pose.position.y + i*0.03
	for j in range(7):
		_temp_pin_pose.pose.position.x = base11.pose.position.x - j*0.03
		pin11_arrayment.append(deepcopy(_temp_pin_pose))

_temp_pin_pose = deepcopy(base14)
pin14_arrayment	= []
for i in range(2):
	_temp_pin_pose.pose.position.y = base14.pose.position.y + i*0.04
	for j in range(3):
		_temp_pin_pose.pose.position.x = base14.pose.position.x - j*0.05
		pin14_arrayment.append(deepcopy(_temp_pin_pose))


_temp_pin_pose = deepcopy(base29)
pin29_arrayment	= []
for i in range(4):
	_temp_pin_pose.pose.position.x = base29.pose.position.x - i*0.05
	pin29_arrayment.append(deepcopy(_temp_pin_pose))


_temp_pin_pose = deepcopy(base26)
pin26_arrayment	= []
for i in range(4):
	_temp_pin_pose.pose.position.x = base26.pose.position.x + i*0.05
	pin26_arrayment.append(deepcopy(_temp_pin_pose))

pin_arrayment.append(pin11_arrayment)
pin_arrayment.append(pin14_arrayment)
pin_arrayment.append(pin26_arrayment)
pin_arrayment.append(pin29_arrayment)