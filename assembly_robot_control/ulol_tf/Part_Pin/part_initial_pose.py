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
part1_pose.pose.position.z = 14 


part2_pose.pose.position.x = -0.31
part2_pose.pose.position.y = 0.08
part2_pose.pose.position.z = 0.81
orientation_list = quaternion_from_euler(0,0,-1.5707)

part2_pose.pose.orientation.x = orientation_list[0]
part2_pose.pose.orientation.y = orientation_list[1]
part2_pose.pose.orientation.z = orientation_list[2]
part2_pose.pose.orientation.w = orientation_list[3]


part3_pose.pose.position.x = -0.15
part3_pose.pose.position.y = 0.04
part3_pose.pose.position.z = 0.81

orientation_list = quaternion_from_euler(0,0,-1.5707)

part3_pose.pose.orientation.x = orientation_list[0]
part3_pose.pose.orientation.y = orientation_list[1]
part3_pose.pose.orientation.z = orientation_list[2]
part3_pose.pose.orientation.w = orientation_list[3]


part4_pose.pose.position.x = -0.74
part4_pose.pose.position.y = -0.30
part4_pose.pose.position.z =  0.82

orientation_list = quaternion_from_euler(0,3.1415,0)
part4_pose.pose.orientation.x = orientation_list[0]
part4_pose.pose.orientation.y = orientation_list[1]
part4_pose.pose.orientation.z = orientation_list[2]
part4_pose.pose.orientation.w = orientation_list[3]


part5_pose.pose.position.x = -0.09
part5_pose.pose.position.y = -0.36
part5_pose.pose.position.z += 0.01
orientation_list = quaternion_from_euler(3.1415,-0.09,-1.5707)
part5_pose.pose.orientation.x = orientation_list[0]
part5_pose.pose.orientation.y = orientation_list[1]
part5_pose.pose.orientation.z = orientation_list[2]
part5_pose.pose.orientation.w = orientation_list[3]


###########################assembly##########################
# part1_pose.pose.position.x = 0
# part1_pose.pose.position.y = 1.05
# part1_pose.pose.position.z = 14 


# part2_pose.pose.position.x = 0.13505834932629283
# part2_pose.pose.position.y = 0.24092942928307576
# part2_pose.pose.position.z = 0.994472440836046
# orientation_list = quaternion_from_euler(-1.5707,1.5707,3.14)

# part2_pose.pose.orientation.x = orientation_list[0]
# part2_pose.pose.orientation.y = orientation_list[1]
# part2_pose.pose.orientation.z = orientation_list[2]
# part2_pose.pose.orientation.w = orientation_list[3]


# part3_pose.pose.position.x = 0.140240265905738004
# part3_pose.pose.position.y = -0.14386830954791996
# part3_pose.pose.position.z = 1.00164543894605

# orientation_list = quaternion_from_euler(1.5707,-1.5707,3.14)

# part3_pose.pose.orientation.x = orientation_list[0]
# part3_pose.pose.orientation.y = orientation_list[1]
# part3_pose.pose.orientation.z = orientation_list[2]
# part3_pose.pose.orientation.w = orientation_list[3]


# part4_pose.pose.position.x = -0.3228479624171441
# part4_pose.pose.position.y = -0.2090361274851515
# part4_pose.pose.position.z =  0.9981496244221348

# orientation_list = quaternion_from_euler(1.5707,1.5707,3.27)
# part4_pose.pose.orientation.x = orientation_list[0]
# part4_pose.pose.orientation.y = orientation_list[1]
# part4_pose.pose.orientation.z = orientation_list[2]
# part4_pose.pose.orientation.w = orientation_list[3]


# part5_pose.pose.position.x = -0.875 
# part5_pose.pose.position.y = 0.020 
# part5_pose.pose.position.z += 0.045
# orientation_list = quaternion_from_euler(3.14,-0.09,0)
# part5_pose.pose.orientation.x = orientation_list[0]
# part5_pose.pose.orientation.y = orientation_list[1]
# part5_pose.pose.orientation.z = orientation_list[2]
# part5_pose.pose.orientation.w = orientation_list[3]


part6_pose.pose.position.x = 0.540
part6_pose.pose.position.y = 0.235 ##0.22
part6_pose.pose.position.z += 0.01
# orientation_list = quaternion_from_euler(3.14,0.09,-1.57)
orientation_list = quaternion_from_euler(3.14,0.0,-1.57)
part6_pose.pose.orientation.x = orientation_list[0]
part6_pose.pose.orientation.y = orientation_list[1]
part6_pose.pose.orientation.z = orientation_list[2]
part6_pose.pose.orientation.w = orientation_list[3]


# part 3 0.034335037575778694, -0.18602806487570622, 0.9721443752681671, -0.4781831250224309, -0.4769069260405939, -0.5203920003177985, 0.52258286313611
# part 2 0.04737980066850862, 0.18964203678913327, 0.9872598452579247, 0.5215923423761206, 0.5228664723037678, -0.47717853509115254, 0.4762905900429556
# part 4 

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