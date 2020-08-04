import rospy

import ulol_UR5 as UR5
import ulol_tf_0730 as UTF

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped

from part_info import*          # part_file_address, part_name

def transrpy2pose(origin,trans,rpy):
	pose = PoseStamped().pose
	pose.position.x = trans[0]
	pose.position.y = trans[1]
	pose.position.z = trans[2]

	olist = quaternion_from_euler(rpy[0],rpy[1],rpy[2])
	pose.orientation.x = olist[0]
	pose.orientation.y = olist[1]
	pose.orientation.w = olist[2]
	pose.orientation.z = olist[3]

	return pose

def init():
	
	try:
		# ASM = ASM_D.Assemble_Data()
		TF = UTF.TF_Node()
		R1 = UR5.UR5()
		R2 = UR5.UR5('rob2_arm')
	

		R1.go_to_initial_pose()
		R2.go_to_initial_pose()
		TF.set_parts([1,2,3,5])
		print "PRES ENTER TO START"
		raw_input()

		TF.change_pin_org(pin_name[0]+"-"+str(1),TF.TF_List[5]['holes'][0])
		TF.change_pin_org(pin_name[0]+"-"+str(2),TF.TF_List[5]['holes'][1])
		TF.change_pin_org(pin_name[0]+"-"+str(3),TF.TF_List[5]['holes'][2])
		TF.change_pin_org(pin_name[0]+"-"+str(4),TF.TF_List[5]['holes'][3])
		TF.change_pin_org(pin_name[0]+"-"+str(5),TF.TF_List[1]['holes'][0])
		TF.change_pin_org(pin_name[0]+"-"+str(6),TF.TF_List[1]['holes'][1])
		# TP1 = transrpy2pose([0.04,-0.16,1.01],[1.57,0,-1.57])
		# TF.change_part_org(1,TP1)
		# TP2 = transrpy2pose([0.06,-0.24,0.98],[1.63,-3.14,1.57])
		# TF.change_part_org(2,TP2)

		TF.assemble_pin(part_name[5],pin_name[0]+"-"+str(1))
		TF.assemble_pin(part_name[5],pin_name[0]+"-"+str(2))
		TF.assemble_pin(part_name[5],pin_name[0]+"-"+str(3))
		TF.assemble_pin(part_name[5],pin_name[0]+"-"+str(4))
		TF.assemble_pin(part_name[1],pin_name[0]+"-"+str(5))
		TF.assemble_pin(part_name[1],pin_name[0]+"-"+str(6))
		TF.assemble_part(part_name[5],part_name[1])
		TF.assemble_part(part_name[5],part_name[2])


		TP = TF.get_grasping_pose(5,2)
	
		R2.grab_part(TP,0.2)
		print R2
		print "PRESS ENTER TO ATTACH"
		raw_input()
		AL = TF.get_attach_list(part_name[5])  

		R2.attach_part(AL)
		print TF.a_list

		print "PRESS ENTER TO UP"
		raw_input()
		R2.add_cartesian_pose([0.1,-0.1,0.1])
		print "PRESS ENTER TO DOWN"
		raw_input()

		R2.add_cartesian_pose([0,0,-0.1])

		print "PRESS ENTER TO DETACH"
		raw_input()
		R2.detach_part(AL)
		TF.detach_part()
		print TF.a_list
		print TF.Pin_List[0]['pose']

		print "PRESS ENTER TO QUIT"
		raw_input()
		R2.go_to_initial_pose()
		

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == '__main__':
	init()