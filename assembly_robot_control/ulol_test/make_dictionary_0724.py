import ulol_wrapper 
import UR5_ulol

def print_dic(dictionary):
	l = len(dictionary)
	print "ik_sol\t",
	for i in range(l):
		print(dictionary.keys()[i]),"\t",
	print""
	for i in range(8):
		print i,"\t",
		for j in range(l):
			print(dictionary.items()[j][1][i]),"\t",
		print ""

def make_dictionary(IKW,target_pose,current_joint):
	trans = [target_pose.position.x,target_pose.position.y,
									target_pose.position.z]
	rot = [target_pose.orientation.x,target_pose.orientation.y,
			target_pose.orientation.z,target_pose.orientation.w]

	inv_sol = IKW._solve(trans,rot)
	IKW.inv_sol = inv_sol
	diff_order = IKW._order_diff(inv_sol,current_joint)

	validity = []
	for i in range(8):
		index = diff_order[i]
		v = IKW._get_state_validity(inv_sol[:,index])
		validity.append(v)
	dictionary = {'diff':diff_order,'validity':validity}
	
	return inv_sol, dictionary

def main():
	R1 = UR5_ulol.UR5()
	IKW = ulol_wrapper.ur5_inv_kin_wrapper()

	pose = R1.move_group.get_current_pose().pose
	pose.position.y -= 0.745
	pose.position.z -= 0.81
	cur_joint = R1.move_group.get_current_joint_values()

	print "ENTER"
	raw_input()

	(inv_sol,DIC) = make_dictionary(IKW,pose,cur_joint)
	print_dic(DIC)
	IKW._print_sol(inv_sol)

	cur_joint = R1.move_group.get_current_joint_values()
	IKW.inv_sol = inv_sol
	for i in range(8):
		print""

		index = DIC['diff'][i]
		if DIC['validity'][i] == True:
			print "#",index," : validity ture"
			raw_input()

			IKW.publish_state(index)

		else:
			print "#",index," : validity false"
			raw_input()

			IKW.publish_state(index)

if __name__ == '__main__':
    main()