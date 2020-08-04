#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import time
import copy

from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from math import pi

from geometry_msgs.msg import *
from tf.transformations import *
from tf import *

import hole_offsets as HO 		# locations of holes of each part
import grasping_point as GP
from part_initial_pose import*	# set initial poses of parts and locations of all pins
from part_info import*			# part_file_address, part_name
from pin_base import*			# base position of pins

class TF_Node(object):

	def __init__(self):
		super(TF_Node, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('ulol_tf_0727', anonymous=True)

		self.scene = moveit_commander.PlanningSceneInterface()
		self.br = TransformBroadcaster()
		self.mesh_added_flag = False
		self.init_TF_List()
		self.init_Pin_List()
		self.init_GP_List()

		rospy.Timer(rospy.Duration(1), self.send_TF)

	def init_Pin_List(self):
		self.Pin_List = [{'name':[],'pose':[]},{'name':[],'pose':[]}
								,{'name':[],'pose':[]},{'name':[],'pose':[]}]
	def init_TF_List(self):
		self.TF_List = [{'origin' : [], 'holes' : []},{'origin' : [],'holes' : []},{'origin' : [],'holes' : []}
						,{'origin' : [],'holes' : []},{'origin' : [],'holes' : []},{'origin' : [],'holes' : []}]
	def init_GP_List(self):
		self.GP_List = [{'pose':[]},{'pose':[]},{'pose':[]},
						{'pose':[]},{'pose':[]},{'pose':[]}]

	def make_orientation_list(self,orientation):
		o_list = [orientation.x, orientation.y, orientation.z, orientation.w]
		return o_list
	def make_position_list(self,position):
		p_list = [position.x,position.y,position.z]
		return p_list

	def part_existence(self,part_num):
		if self.TF_List[part_num]['origin'] == []:
			return False
		else:
			return True
	def Pose_to_PoseStamped(self,pose):
		if type(pose) == geometry_msgs.msg._PoseStamped.PoseStamped:
			return pose
		else:
			pose_stamped = PoseStamped()
			pose_stamped.pose = copy.deepcopy(pose)
			return pose_stamped

	def add_mesh(self, mesh_name, file_name, mesh_pose):
		mesh_pose.header.frame_id = "world"
		position_list = self.make_position_list(mesh_pose.pose.position)
		orientation_list = self.make_orientation_list(mesh_pose.pose.orientation)
		self.scene.add_mesh(mesh_name, mesh_pose, file_name, size=(1, 1, 1))
		self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), mesh_name, 'world')
		# print "[INFO] MESH NAME = '", mesh_name,"' ADDED"
	def add_pin(self, pin_type):
		#pin_number is its index in the list
		pin_exist = len(self.Pin_List[pin_type]['pose'])
		pin_pose = pin_arrayment[pin_type][pin_exist]

		Pin_name = pin_name[pin_type]+"-"+str(pin_exist+1)

		self.add_mesh(Pin_name,pin_file[pin_type],pin_pose)
		if pin_has_hole[pin_type]:
			pin_origin_pose = copy.deepcopy(pin_pose.pose)

			pin_hole_offset = HO.pin_hole_offset[pin_type]
			pin_hole_pose = self.get_hole_pose(pin_origin_pose,pin_type,0,pin_hole_offset)

			pin_pose = copy.deepcopy({'pin':pin_origin_pose,'hole':pin_hole_pose})
		else:
			pin_pose = copy.deepcopy(pin_pose.pose)
		self.Pin_List[pin_type]['pose'].append(pin_pose)

	def change_part_org(self,part_num,new_pose):
		self.mesh_added_flag == False

		new_pose_stamped = self.Pose_to_PoseStamped(new_pose)
		self.add_mesh(part_name[part_num],part_file[part_num],new_pose_stamped)

		self.TF_List[part_num]['origin'] = copy.deepcopy(new_pose_stamped.pose)
		self.TF_List[part_num]['holes'] = []
		num_of_holes = len(HO.hole_offset[part_num])
		for hole_num in range(num_of_holes):
			hole_data = HO.hole_offset[part_num][hole_num]
			hole_pose = self.get_hole_pose(new_pose_stamped.pose,part_num,hole_num,hole_data)
			self.TF_List[part_num]['holes'].append(copy.deepcopy(hole_pose))

		self.GP_List[part_num]['pose'] = []
		num_of_gp = len(GP.grasping_pose[part_num])
		for gp in range(num_of_gp):
			grasping_pose = self.get_grasping_pose(part_num,gp)
			self.GP_List[part_num]['pose'].append(copy.deepcopy(grasping_pose))

		self.mesh_added_flag == True

	def move_part_org(self,part_num,trans,rot):
		self.mesh_added_flag == False

		if rot != [0,0,0]:
			old_org = copy.deepcopy(self.TF_List[part_num]['origin'])
			old_olist = self.make_orientation_list(old_org.orientation)
			old_rot = euler_from_quaternion(old_olist)
			old_R = euler_matrix(old_rot[0],old_rot[1],old_rot[2])
			add_R = euler_matrix(rot[0],rot[1],rot[2])
			old_R = old_R.dot(add_R)
			new_rot = copy.deepcopy(euler_from_matrix(old_R))
			new_olist = quaternion_from_euler(new_rot[0],new_rot[1],new_rot[2])
			self.TF_List[part_num]['origin'].orientation.x = new_olist[0] 
			self.TF_List[part_num]['origin'].orientation.y = new_olist[1]
			self.TF_List[part_num]['origin'].orientation.z = new_olist[2]
			self.TF_List[part_num]['origin'].orientation.w = new_olist[3]

		self.TF_List[part_num]['origin'].position.x += trans[0] 
		self.TF_List[part_num]['origin'].position.y += trans[1]
		self.TF_List[part_num]['origin'].position.z += trans[2]

		part_org = self.TF_List[part_num]['origin']

		new_pose_stamped = self.Pose_to_PoseStamped(part_org)
		self.add_mesh(part_name[part_num],part_file[part_num],new_pose_stamped)

		self.TF_List[part_num]['holes'] = []
		num_of_holes = len(HO.hole_offset[part_num])
		for hole_num in range(num_of_holes):
			hole_data = HO.hole_offset[part_num][hole_num]
			hole_pose = self.get_hole_pose(part_org,part_num,hole_num,hole_data)
			self.TF_List[part_num]['holes'].append(copy.deepcopy(hole_pose))

		self.GP_List[part_num]['pose'] = []
		num_of_gp = len(GP.grasping_pose[part_num])
		for gp in range(num_of_gp):
			grasping_pose = self.get_grasping_pose(part_num,gp)
			self.GP_List[part_num]['pose'].append(copy.deepcopy(grasping_pose))

		self.mesh_added_flag == True

	def get_hole_pose(self,origin_pose,part_num,hole_num,hole_offset):
		#origin pose : pose of origin of part : mesh_pose.pose

		o_list_origin = copy.deepcopy(self.make_orientation_list(origin_pose.orientation))
		
		Rotation = quaternion_matrix(o_list_origin)

		axis_x = Rotation[0:3,0]
		axis_y = Rotation[0:3,1]
		axis_z = Rotation[0:3,2]

		hole_pose = geometry_msgs.msg.PoseStamped().pose
		hole_pose.position = copy.deepcopy(origin_pose.position)

		x_offset = axis_x[0]*hole_offset['trans'][0] + axis_y[0]*hole_offset['trans'][1] + axis_z[0]*hole_offset['trans'][2]
		y_offset = axis_x[1]*hole_offset['trans'][0] + axis_y[1]*hole_offset['trans'][1] + axis_z[1]*hole_offset['trans'][2]
		z_offset = axis_x[2]*hole_offset['trans'][0] + axis_y[2]*hole_offset['trans'][1] + axis_z[2]*hole_offset['trans'][2]

		hole_pose.position.x += x_offset
		hole_pose.position.y += y_offset
		hole_pose.position.z += z_offset

		rol_add = hole_offset['rot'][0]
		pit_add = hole_offset['rot'][1]
		yaw_add = hole_offset['rot'][2]

		Rotation_add = euler_matrix(rol_add,pit_add,yaw_add)
		Rotation_new = Rotation.dot(Rotation_add)

		(rol,pit,yaw) = euler_from_matrix(Rotation_new)
		o_list = quaternion_from_euler(rol,pit,yaw)
		hole_pose.orientation.x = o_list[0]
		hole_pose.orientation.y = o_list[1]
		hole_pose.orientation.z = o_list[2]
		hole_pose.orientation.w = o_list[3]

		return hole_pose

	def get_grasping_pose(self,part_num,point_num):
		#point_num is grasping point number

		part_org = self.TF_List[part_num]['origin']
		o_list_origin = copy.deepcopy(self.make_orientation_list(part_org.orientation))
		
		Rotation = quaternion_matrix(o_list_origin)

		axis_x = Rotation[0:3,0]
		axis_y = Rotation[0:3,1]
		axis_z = Rotation[0:3,2]

		grasp_pose = geometry_msgs.msg.PoseStamped().pose
		grasp_pose.position = copy.deepcopy(part_org.position)

		offset = GP.grasping_pose[part_num][point_num]

		x_offset = axis_x[0]*offset['trans'][0] + axis_y[0]*offset['trans'][1] + axis_z[0]*offset['trans'][2]
		y_offset = axis_x[1]*offset['trans'][0] + axis_y[1]*offset['trans'][1] + axis_z[1]*offset['trans'][2]
		z_offset = axis_x[2]*offset['trans'][0] + axis_y[2]*offset['trans'][1] + axis_z[2]*offset['trans'][2]

		grasp_pose.position.x += x_offset
		grasp_pose.position.y += y_offset
		grasp_pose.position.z += z_offset


		# (rol,pit,yaw) = euler_from_quaternion(o_list_origin)
		rol_add = offset['rot'][0]
		pit_add = offset['rot'][1]
		yaw_add = offset['rot'][2]

		Rotation_add = euler_matrix(rol_add,pit_add,yaw_add)

		Rotation_new = Rotation.dot(Rotation_add)
		(rol,pit,yaw) = euler_from_matrix(Rotation_new)

		o_list = quaternion_from_euler(rol,pit,yaw)
		grasp_pose.orientation.x = o_list[0]
		grasp_pose.orientation.y = o_list[1]
		grasp_pose.orientation.z = o_list[2]
		grasp_pose.orientation.w = o_list[3]

		return grasp_pose

	def set_parts(self):
		for p_num in [1,2,3,5]:
			time.sleep(0.3)
			self.add_mesh(part_name[p_num],part_file[p_num],part_pose[p_num])
			self.set_part_TF(part_name[p_num])
		
		for i in range(4):
			for j in range(how_many_pins[i]):
				self.add_pin(i)


		self.mesh_added_flag = True
	
	def set_part_TF(self,mesh_name): # fill TF_list and GP_list
		part_num = part_name.index(mesh_name)
		mesh_pose = geometry_msgs.msg.PoseStamped()
		mesh_pose.header = self.scene.get_objects([mesh_name])[mesh_name].header
		mesh_pose.pose = self.scene.get_objects([mesh_name])[mesh_name].mesh_poses[0]

		self.TF_List[part_num]['origin']  = copy.deepcopy(mesh_pose.pose)
		
		num_of_holes = len(HO.hole_offset[part_num])
		for hole_num in range(num_of_holes):

			hole_data = HO.hole_offset[part_num][hole_num]
			hole_pose = self.get_hole_pose(mesh_pose.pose,part_num,hole_num,hole_data)
			self.TF_List[part_num]['holes'].append(copy.deepcopy(hole_pose))

		num_of_gp = len(GP.grasping_pose[part_num])
		for gp in range(num_of_gp):
			grasping_pose = self.get_grasping_pose(part_num,gp)
			self.GP_List[part_num]['pose'].append(copy.deepcopy(grasping_pose))

	def set_pin_TF(self,pin_type,pin_tag): # fill Pin_lsit #Pin_name = pin_name[pin_number]+"-"+str(pin_exist+1)

		target_pin_name = pin_name[pin_type]+"-"+str(pin_tag+1)
		tarset_pin_TF = geometry_msgs.msg.PoseStamped()
		tarset_pin_TF.header = self.scene.get_objects([target_pin_name])[target_pin_name].header
		tarset_pin_TF.pose = self.scene.get_objects([target_pin_name])[target_pin_name].mesh_poses[0]

		if pin_has_hole[pin_type]:
			pin_origin_pose = copy.deepcopy(tarset_pin_TF.pose)
			
			pin_hole_offset = HO.pin_hole_offset[pin_type]
			pin_hole_pose = self.get_hole_pose(pin_origin_pose,pin_type,0,pin_hole_offset)

			pin_pose = copy.deepcopy({'pin':pin_origin_pose,'hole':pin_hole_pose})

		else:
			pin_pose = copy.deepcopy(tarset_pin_TF.pose)
		self.Pin_List[pin_type]['pose'].append(pin_pose)

	def send_part_TF(self):
		for p_num in range(6):
			if self.part_existence(p_num):
				selected_part = self.TF_List[p_num]
				position_list = self.make_position_list(selected_part['origin'].position)
				orientation_list = self.make_orientation_list(selected_part['origin'].orientation)

				self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), part_name[p_num], 'world')

				num_of_holes = len(HO.hole_offset[p_num])

				for h_num in range(num_of_holes):
					hole_name = "hole" + str(p_num+1) + "-" + str(h_num+1)

					position_list = self.make_position_list(selected_part['holes'][h_num].position)
					orientation_list = self.make_orientation_list(selected_part['holes'][h_num].orientation)

					self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), hole_name, 'world')

	def send_pin_TF(self):
		for pin_type in range(4):
			_pin_exist = len(self.Pin_List[pin_type]['pose'])
			selected_pin = self.Pin_List[pin_type]

			for pins in range(_pin_exist):
				if pin_has_hole[pin_type] == True:
					Pin_name = pin_name[pin_type]+"-"+str(pins)
					Hole_name = Pin_name+"hole"
					#firstly pin
					position_list = self.make_position_list(selected_pin['pose'][pins]['pin'].position)
					orientation_list = self.make_orientation_list(selected_pin['pose'][pins]['pin'].orientation)
					self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), Pin_name, 'world')

					#secondary hole
					position_list = self.make_position_list(selected_pin['pose'][pins]['hole'].position)
					orientation_list = self.make_orientation_list(selected_pin['pose'][pins]['hole'].orientation)
					self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), Hole_name, 'world')
				else:
					Pin_name = pin_name[pin_type]+"-"+str(pins)
					position_list = self.make_position_list(selected_pin['pose'][pins].position)
					orientation_list = self.make_orientation_list(selected_pin['pose'][pins].orientation)
					self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), Pin_name, 'world')

	def send_GP_TF(self):
		for part_type in range(6):
			num_of_gp = len(self.GP_List[part_type]['pose'])
			selected_part = self.GP_List[part_type]
			for g in range(num_of_gp):
				gp_name = part_name[part_type]+"-GRASP-"+str(g+1)
				position_list = self.make_position_list(selected_part['pose'][g].position)
				orientation_list = self.make_orientation_list(selected_part['pose'][g].orientation)
				self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), gp_name, 'world')

	def send_TF(self,dumb_Data):
		if self.mesh_added_flag == True:
			# print"[INFO] send_TF CALLED"
			self.send_part_TF()
			self.send_pin_TF()
			self.send_GP_TF()	



	def update_tf(self):
		self.mesh_added_flag = False

		self.init_TF_List()
		self.init_Pin_List()
		self.init_GP_List()

		for p_num in [1,2,3,5]:
			time.sleep(0.3)
			self.set_part_TF(part_name[p_num])

		for pin_type in range(4):
			for pin_tag in range(how_many_pins[pin_type]):
				self.set_pin_TF(pin_type,pin_tag)


		self.mesh_added_flag = True



def init():



	TF = TF_Node()
	TF.set_parts()

	try:
		print "Press ENTER to move change_part2 to y + 0.1"
		raw_input()
		TF.update_tf()
		
		TF.move_part_org(part_num =1,trans = [0,0.1,0],rot = [0,0,0])
		print "Press ENTER to move change_part2 to z + 0.1"
		raw_input()
		TF.move_part_org(part_num =1,trans = [0,0,0.1],rot = [0,0,0])
		print "Press ENTER to move change_part2 to rz + pi/2"
		raw_input()
		TF.move_part_org(part_num =1,trans = [0,0,0],rot = [0,0,1.57])
		print "Press ENTER to move change_part2 initial origin"
		raw_input()
		TF.change_part_org(part_num =1,new_pose = part_pose[1])

		print "Press ENTER to quit"
		raw_input()

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == '__main__':
	init()