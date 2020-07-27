import ulol_wrapper 
import UR5_ulol

def print_dic(dictionary):
	l = len(dictionary)
	for i in range(l):
		print(dictionary.keys()[i]),
	print""
	for i in range(8):
		for j in range(l):
			print(dictionary.items()[j][1][i]),
		print ""

def make_dictionary(IKW,target_pose,current_joint):
	trans = [target_pose.position.x,target_pose.position.y,
									target_pose.position.z]
	rot = [target_pose.orientation.x,target_pose.orientation.y,
			target_pose.orientation.z,target_pose.orientation.w]

	inv_sol = IKW._solve(trans,rot)
	diff_order = IKW._order_diff(inv_sol,current_joint)

	validity = []
	for i in range(8):
		v = IKW._get_state_validity(inv_sol[:,i])
		validity.append(v)
	dictionary = {'diff':diff_order,'validity':validity}
	
	print_dic(dictionary)

def main():
	R1 = UR5_ulol.UR5()
	IKW = ulol_wrapper.ur5_inv_kin_wrapper()

	pose = R1.move_group.get_current_pose().pose
	cur_joint = R1.move_group.get_current_joint_values()

	DIC = make_dictionary(IKW,pose,cur_joint)
	print DIC

if __name__ == '__main__':
    main()