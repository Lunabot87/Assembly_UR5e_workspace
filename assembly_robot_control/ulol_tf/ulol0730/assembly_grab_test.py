import rospy

import assemble_data as ASM_D
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
		TF = UTF.TF_Node()
		R1 = UR5.UR5()
		R2 = UR5.UR5('rob2_arm')
		ASM = ASM_D.Assemble_Data()

		R1.go_to_initial_pose()
		R2.go_to_initial_pose()
		TF.set_parts()

		print "PRES ENTER TO START"
		raw_input()

		TF.change_pin_org(pin_name[0]+"-"+str(1),TF.TF_List[5]['holes'][0])
		TF.change_pin_org(pin_name[0]+"-"+str(2),TF.TF_List[5]['holes'][1])
		TF.change_pin_org(pin_name[0]+"-"+str(3),TF.TF_List[5]['holes'][2])
		TF.change_pin_org(pin_name[0]+"-"+str(4),TF.TF_List[5]['holes'][3])
		TF.change_pin_org(pin_name[0]+"-"+str(5),TF.TF_List[5]['holes'][4])
		TF.change_pin_org(pin_name[0]+"-"+str(6),TF.TF_List[5]['holes'][5])
		TF.update_tf()
		# TP1 = transrpy2pose([0.04,-0.16,1.01],[1.57,0,-1.57])

		# TF.change_part_org(1,TP1)
		
		# TP2 = transrpy2pose([0.06,-0.24,0.98],[1.63,-3.14,1.57])

		# TF.change_part_org(2,TP2)

		ASM.assemble_pin(part_name[5],pin_name[0]+"-"+str(1))
		ASM.assemble_pin(part_name[5],pin_name[0]+"-"+str(2))
		ASM.assemble_pin(part_name[5],pin_name[0]+"-"+str(3))
		ASM.assemble_pin(part_name[5],pin_name[0]+"-"+str(4))
		ASM.assemble_pin(part_name[1],pin_name[0]+"-"+str(5))
		ASM.assemble_pin(part_name[1],pin_name[0]+"-"+str(6))
		ASM.assemble_part(part_name[5],part_name[1])
		ASM.assemble_part(part_name[5],part_name[2])


		TP = TF.get_grasping_pose(5,1)
		print TP
		AL = ASM.get_attach_list(part_name[5])
		print AL

		R2.grab_part(AL,TP,0.2)

		R2.go_to_initial_pose()

		TF.update_tf()
		print "PRESS ENTER TO QUIT"
		raw_input()

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == '__main__':
	init()