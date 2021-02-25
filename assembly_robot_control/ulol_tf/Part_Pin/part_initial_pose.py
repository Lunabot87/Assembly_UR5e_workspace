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

# part2_pose.pose.position.x = -0.26
# part2_pose.pose.position.y = 0.15
# part2_pose.pose.position.z = 0.81-0.008
# orientation_list = quaternion_from_euler(0,0,3.14)

# part2_pose.pose.orientation.x = orientation_list[0]
# part2_pose.pose.orientation.y = orientation_list[1]
# part2_pose.pose.orientation.z = orientation_list[2]
# part2_pose.pose.orientation.w = orientation_list[3]

# part3_pose.pose.position.x = -0.25
# part3_pose.pose.position.y = -0.02
# part3_pose.pose.position.z = 0.81-0.008
# orientation_list = quaternion_from_euler(0,0,3.14)
# part3_pose.pose.orientation.x = orientation_list[0]
# part3_pose.pose.orientation.y = orientation_list[1]
# part3_pose.pose.orientation.z = orientation_list[2]
# part3_pose.pose.orientation.w = orientation_list[3]

# part4_pose.pose.position.x = -0.4
# part4_pose.pose.position.y = -0.62
# part4_pose.pose.position.z += 0.01
# #orientation_list = quaternion_from_euler(3.1415,0,-1.5707)
# orientation_list = quaternion_from_euler(3.1415,0,3.1415)
# part4_pose.pose.orientation.x = orientation_list[0]
# part4_pose.pose.orientation.y = orientation_list[1]
# part4_pose.pose.orientation.z = orientation_list[2]
# part4_pose.pose.orientation.w = orientation_list[3]



# part2_pose.pose.position.x = 0.03675834932629283
# part2_pose.pose.position.y = 0.21092942928307576
# part2_pose.pose.position.z = 0.9894472440836046

# part2_pose.pose.orientation.x = 0.520625012000874
# part2_pose.pose.orientation.y = 0.5225031037636205
# part2_pose.pose.orientation.z = -0.47806352808006214
# part2_pose.pose.orientation.w = 0.4768599024410808


# part3_pose.pose.position.x = 0.042240265905738004
# part3_pose.pose.position.y = -0.17386830954791996
# part3_pose.pose.position.z = 0.9749164543894605
# part3_pose.pose.orientation.x = -0.4782755344218218
# part3_pose.pose.orientation.y = -0.47704198767220163
# part3_pose.pose.orientation.z = -0.5202035702659914
# part3_pose.pose.orientation.w = 0.5225626284511662


part2_pose.pose.position.x = 0.13505834932629283
part2_pose.pose.position.y = 0.24092942928307576
part2_pose.pose.position.z = 0.994472440836046
orientation_list = quaternion_from_euler(-1.5707,1.5707,3.14)

part2_pose.pose.orientation.x = orientation_list[0]
part2_pose.pose.orientation.y = orientation_list[1]
part2_pose.pose.orientation.z = orientation_list[2]
part2_pose.pose.orientation.w = orientation_list[3]


part3_pose.pose.position.x = 0.140240265905738004
part3_pose.pose.position.y = -0.14386830954791996
part3_pose.pose.position.z = 1.00164543894605

orientation_list = quaternion_from_euler(1.5707,-1.5707,3.14)

part3_pose.pose.orientation.x = orientation_list[0]
part3_pose.pose.orientation.y = orientation_list[1]
part3_pose.pose.orientation.z = orientation_list[2]
part3_pose.pose.orientation.w = orientation_list[3]


part4_pose.pose.position.x = -0.3228479624171441
part4_pose.pose.position.y = -0.2090361274851515
part4_pose.pose.position.z =  0.9981496244221348

orientation_list = quaternion_from_euler(1.5707,1.5707,3.27)
part4_pose.pose.orientation.x = orientation_list[0]
part4_pose.pose.orientation.y = orientation_list[1]
part4_pose.pose.orientation.z = orientation_list[2]
part4_pose.pose.orientation.w = orientation_list[3]


# part4_pose.pose.position.x = -0.4128479624171441
# part4_pose.pose.position.y = -0.2690361274851515
# part4_pose.pose.position.z =  0.9661496244221348

# part4_pose.pose.orientation.x = -0.517128037887288
# part4_pose.pose.orientation.y = 0.43354511756533726
# part4_pose.pose.orientation.z = 0.5599683644537544
# part4_pose.pose.orientation.w = 0.4806793674343527

# part5_pose.pose.position.x = 0.460
# part5_pose.pose.position.y = 0.20
# part5_pose.pose.position.z += 0.38
# orientation_list = quaternion_from_euler(3.14,0.09,-1.57)
# part5_pose.pose.orientation.x = orientation_list[0]
# part5_pose.pose.orientation.y = orientation_list[1]
# part5_pose.pose.orientation.z = orientation_list[2]
# part5_pose.pose.orientation.w = orientation_list[3]



# part5_pose.pose.position.x = 1.05
# part5_pose.pose.position.y = -0.05
# part5_pose.pose.position.z += 0.01
# orientation_list = quaternion_from_euler(3.14,-0.09,-1.57)
# part5_pose.pose.orientation.x = orientation_list[0]
# part5_pose.pose.orientation.y = orientation_list[1]
# part5_pose.pose.orientation.z = orientation_list[2]
# part5_pose.pose.orientation.w = orientation_list[3]

part5_pose.pose.position.x = 0.540
part5_pose.pose.position.y = 0.235
part5_pose.pose.position.z += 0.37
orientation_list = quaternion_from_euler(3.14,0.00,-1.57)
part5_pose.pose.orientation.x = orientation_list[0]
part5_pose.pose.orientation.y = orientation_list[1]
part5_pose.pose.orientation.z = orientation_list[2]
part5_pose.pose.orientation.w = orientation_list[3]




part6_pose.pose.position.x = 0.540
part6_pose.pose.position.y = 0.235 ##0.22
part6_pose.pose.position.z += 0.01
# orientation_list = quaternion_from_euler(3.14,0.09,-1.57)
orientation_list = quaternion_from_euler(3.14,0.0,-1.57)
part6_pose.pose.orientation.x = orientation_list[0]
part6_pose.pose.orientation.y = orientation_list[1]
part6_pose.pose.orientation.z = orientation_list[2]
part6_pose.pose.orientation.w = orientation_list[3]







# part6_pose.pose.position.x = 0.430
# part6_pose.pose.position.y = 0.22 ##0.22
# part6_pose.pose.position.z += 0.01
# # orientation_list = quaternion_from_euler(3.14,0.09,-1.57)
# orientation_list = quaternion_from_euler(3.14,0.0,-1.57)
# part6_pose.pose.orientation.x = orientation_list[0]
# part6_pose.pose.orientation.y = orientation_list[1]
# part6_pose.pose.orientation.z = orientation_list[2]
# part6_pose.pose.orientation.w = orientation_list[3]

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