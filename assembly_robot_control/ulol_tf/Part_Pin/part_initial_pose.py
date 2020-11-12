from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from tf.transformations import quaternion_from_euler
from pin_base import pin_base11, pin_base14, pin_base26, pin_base29

part1_pose = PoseStamped()
part2_pose = PoseStamped()
part3_pose = PoseStamped()
part4_pose = PoseStamped()
part5_pose = PoseStamped()
part6_pose = PoseStamped()

part_init_pose = [part1_pose,part2_pose,part3_pose,part4_pose,part5_pose,part6_pose]

for i in range(6):
	part_init_pose[i].header.frame_id = "world"
	part_init_pose[i].pose.orientation.w = 1.0
	part_init_pose[i].pose.position.z = 0.81

part1_pose.pose.position.x = 0
part1_pose.pose.position.y = 1.05

part2_pose.pose.position.x = -0.26
part2_pose.pose.position.y = 0.13
part2_pose.pose.position.z -= 0
orientation_list = quaternion_from_euler(0,0,3.14)

# part2_pose.pose.position.x = 0.040
# part2_pose.pose.position.y = 0.226
# part2_pose.pose.position.z += 0.20
# orientation_list = quaternion_from_euler(1.502,1.551,-0.068)
part2_pose.pose.orientation.x = orientation_list[0]
part2_pose.pose.orientation.y = orientation_list[1]
part2_pose.pose.orientation.z = orientation_list[2]
part2_pose.pose.orientation.w = orientation_list[3]

part3_pose.pose.position.x = -0.25
part3_pose.pose.position.y = -0.02
part3_pose.pose.position.z -= 0
orientation_list = quaternion_from_euler(0,0,3.14)


# part3_pose.pose.position.x = 0.046 
# part3_pose.pose.position.y = -0.167 
# part3_pose.pose.position.z =  0.971 
# orientation_list = quaternion_from_euler(-0.036, -1.481, -1.493)

part3_pose.pose.orientation.x = orientation_list[0]
part3_pose.pose.orientation.y = orientation_list[1]
part3_pose.pose.orientation.z = orientation_list[2]
part3_pose.pose.orientation.w = orientation_list[3]

part4_pose.pose.position.x = -0.5
part4_pose.pose.position.y = -0.8
part4_pose.pose.position.z += 0.01
orientation_list = quaternion_from_euler(3.1415,0,-1.5707)
part4_pose.pose.orientation.x = orientation_list[0]
part4_pose.pose.orientation.y = orientation_list[1]
part4_pose.pose.orientation.z = orientation_list[2]
part4_pose.pose.orientation.w = orientation_list[3]

part5_pose.pose.position.x = 0.5
part5_pose.pose.position.y = -1.0
part5_pose.pose.position.z += 0.01
orientation_list = quaternion_from_euler(3.14,-0.09,-1.57)
part5_pose.pose.orientation.x = orientation_list[0]
part5_pose.pose.orientation.y = orientation_list[1]
part5_pose.pose.orientation.z = orientation_list[2]
part5_pose.pose.orientation.w = orientation_list[3]

part6_pose.pose.position.x = 0.430
part6_pose.pose.position.y = 0.24
part6_pose.pose.position.z += 0.01
orientation_list = quaternion_from_euler(3.14,0.09,-1.57)
part6_pose.pose.orientation.x = orientation_list[0]
part6_pose.pose.orientation.y = orientation_list[1]
part6_pose.pose.orientation.z = orientation_list[2]
part6_pose.pose.orientation.w = orientation_list[3]


pin_arrayment = []

_temp_pin_pose = deepcopy(pin_base11)
pin11_arrayment	= []
for i in range(3):
	_temp_pin_pose.pose.position.y = pin_base11.pose.position.y + i*0.03
	for j in range(7):
		_temp_pin_pose.pose.position.x = pin_base11.pose.position.x - j*0.03
		pin11_arrayment.append(deepcopy(_temp_pin_pose))

_temp_pin_pose = deepcopy(pin_base14)
pin14_arrayment	= []
for i in range(2):
	_temp_pin_pose.pose.position.y = pin_base14.pose.position.y + i*0.04
	for j in range(3):
		_temp_pin_pose.pose.position.x = pin_base14.pose.position.x - j*0.05
		pin14_arrayment.append(deepcopy(_temp_pin_pose))


_temp_pin_pose = deepcopy(pin_base29)
pin29_arrayment	= []
for i in range(4):
	_temp_pin_pose.pose.position.x = pin_base29.pose.position.x - i*0.05
	pin29_arrayment.append(deepcopy(_temp_pin_pose))


_temp_pin_pose = deepcopy(pin_base26)
pin26_arrayment	= []
for i in range(4):
	_temp_pin_pose.pose.position.x = pin_base26.pose.position.x + i*0.05
	pin26_arrayment.append(deepcopy(_temp_pin_pose))

pin_arrayment.append(pin11_arrayment)
pin_arrayment.append(pin14_arrayment)
pin_arrayment.append(pin26_arrayment)
pin_arrayment.append(pin29_arrayment)


# Consequently, part_pose and pin_arrayment