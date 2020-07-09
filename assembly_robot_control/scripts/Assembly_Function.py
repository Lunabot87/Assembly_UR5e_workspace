

class Assembly_mode():
	def __init__():

	def insert_pin(asm_msg):
		# (일단은) 모두 rob1 이 작업 
		real_insert_target_pose = fine_tune_insert_target(asm_msg.parent.target) # pin일 때는 parent 타겟이 항상 하나
		grab_pin(asm_msg.child, False)
		insert_spiral_motion(real_insert_target_pose)

	def insert_part(asm_msg):
		# rob1, rob2 작업, rob1이 작업 중심
		sorted_insert_target_poses = sort_insert_target(asm_msg.parents)
		## move() - parent part는 고정, child part가 rob1이 작업을 할 수 있는 위치에 없으면 할수 있는 위치로 옮긴다
		is_moved = hand_over_part(sorted_insert_target_poses, asm_msg)
		grab_part(asm_msg.child, is_moved)
		insert_part_motion(sorted_insert_target_poses[0])



class Assembly_process():
	def __init__(self):
		self.params = get_param_from_grasp_yaml()

	def fine_tune_insert_target():
		# target_pose[PoseStamped] : 핀을 꽂은 상태에서 eef의 목표 값
		# kHoleCheckOffset
		return target_pose

	def hand_over_part(self, insert_target_pose, asm_msg):
		# 1. 낄 수 있는지 2. 들 수 있는지 확인
		# child part를 낄 수 있는지 확인해서 못끼면, 로봇을 변경
		# 바뀐 로봇이 child part를 들 수 있는지 확인해서 못들면, child part를 옮긴다.
		# 끼는 target 위치는 insert 할 위치
		# 드는 target 위치는 child part의 위치
		# 작업하는 로봇이 바뀐 경우 True 반환, 그대로인 경우 False 반환
		rob = rob1
		# rob1이 낄 수 있거나 rob2가 낄수 있거나, 둘다 안되는 경우는 발생 x 가정
		if !check_reachability(insert_target_pose, rob):
			rob = rob2 
		if !check_reachability(asm_msg.child.pose, rob):
			pass_part_to_other_rob(asm_msg.child.pose, COMMON_AREA_TARGET_POSE, rob)
			return True
		return False

	def grab_pin(asm_child_msg, is_moved):
		grasp = make_grasp_msg(asm_child_msg.pin, asm_child_msg.pose)
		pick_up(grasp)

	def grab_part(asm_child_msg, is_moved):
		if is_moved is True:
			grasp = make_grasp_msg(asm_child_msg.part)
		else:
			grasp = make_grasp_msg(asm_child_msg.part, asm_child_msg.pose)
		pick_up(grasp)

	def insert_spiral_motion(num_of_trial=5, real_insert_target_pose):
		# spiral() 실행, 성공할 때까지 num_of_trial 만큼 반복
		# target pose와 grasp_config.yaml 의 데이터를 합쳐서 approach, retreat도 결정 
		for i in range(num_of_trial):
			if sprial_motion(real_insert_target_pose):
				break

	def insert_part_motion(sorted_insert_target_poses):
		## ??? 
		pass

	def sort_insert_target(asm_parents_msg):
		groups = dict()
		singles = []
		for parent in len(asm_parents_msg):
			if parent.group_name is not '':
				if parent.group_name in gruops:
					groups[parent.group_name].append(parent.target)
				else:
					groups[parent.group_name] = [parent.target]
			else:
				singles.append(parent.target)

		insert_sorted_poses = sort_priority(groups, singles) 
		return insert_sorted_poses

	def sort_priority(groups, singles):
		# priority는 single 인지 group 인지를 기준으로 설정
		# 추후 child를 잡고 있는 위치 값도 들어가야 함(무게중심의 기준값으로 친다)
		pass

	def make_grasp_msg(part_name, child_frame_pose):
		# child_frame_pose가 들어오면, part_name을 pick_config.yaml에서 찾아서
		# 해당하는 part_grasp_target_pose 값과 child_frame_pose를 합쳐서 grasping pose(robot base frame 기준) 결정
		# child_frame_pose가 들어오지 않으면, pick_config.yaml에서 
		# common_area_grasp_target_pose(world frame 기준)로 grasping pose 결정
		global_child_pose = child_frame_pose
		self.params[part_name].target_pose
		self.params[part_name].offset		
		return grasp

	def make_insert_msg(part_name, child_frame_pose):
		pass



class Assembly_motion():
	def __init__():
		self.group1 = moveit_commander.MoveGroupCommander("rob1")
		self.group2 = moveit_commander.MoveGroupCommander("rob2")
		self.rob1 = urx.Robot('''rob1_ip''')
		self.rob2 = urx.Robot('''rob2_ip''')

	def pick_up(grasp):
		self.group1.go(grasp.pre_grasp)
		self.group1.go(grasp.grasp)
		self.group1.go(grasp.post_grasp)

	def move_to(target_pose):
	def sprial_motion():
		# spiral motion을 진행하면서 pin을 insert 하는 작업
		# force값을 받아서 pin이 insert가 되었는지 아닌지 확인
		# insert가 되면 모션 중지하고 True를 반환
		# motion을 끝까지 진행하였는데도 insert가 안되었으면 False를 반환
		return is_inserted