#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import time
import copy
import numpy

from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from math import pi

from geometry_msgs.msg import *
from tf.transformations import *
from tf import *

from Part_Pin.part_initial_pose import *# part_init_pose, pin_arrayment
from Part_Pin.part_ready_pose import *	# part_ready_pose
from Part_Pin.part_spare_pose import *
from Part_Pin.part12_assembling_pose import *
from Part_Pin.part_info import*			# part_file_address, part_name
from Part_Pin.pin_base import pin_TF_pose


import Part_Pin.hole_offsets as HO 		# locations of holes of each part
import Part_Pin.grasping_point as GP

def make_orientation_list(orientation):
    o_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    return o_list
def make_position_list(position):
    p_list = [position.x,position.y,position.z]
    return p_list
def pose_from_transrot(trans = [0,0,0],rot = [0,0,0]):
    pose = geometry_msgs.msg.PoseStamped().pose
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]

    o_list = quaternion_from_euler(rot[0],rot[1],rot[2])
    pose.orientation.x = o_list[0]
    pose.orientation.y = o_list[1]
    pose.orientation.z = o_list[2]
    pose.orientation.w = o_list[3]

    return pose
def transrot_from_pose(pose):
	trans = [pose.position.x,pose.position.y,pose.position.z]
	o_list = make_orientation_list(pose.orientation)
	rot = euler_from_quaternion(o_list)
	return (trans,rot)
def transquat_from_pose(pose):
	trans = [pose.position.x,pose.position.y,pose.position.z]
	o_list = make_orientation_list(pose.orientation)
	return (trans,o_list)
def mat_from_pose(pose):
	quat = make_orientation_list(pose.orientation)
	trans = make_position_list(pose.position)
	mat = quaternion_matrix(quat)
	mat[0:3,3] = trans
	return mat
def pose_from_mat(mat):
	rpy = euler_from_matrix(mat)
	trans = mat[0:3,3]
	pose = pose_from_transrot(trans,rpy)
	return pose

class TF_Node(object):

	def __init__(self):
		super(TF_Node, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		self.scene = moveit_commander.PlanningSceneInterface()
		self.br = TransformBroadcaster()

		self.init_TF_List()
		self.init_Pin_List()
		self.init_GP_List()
		self.init_ASM_list()
		self.init_attach_list()

		self.part_add_flag = False
		
		rospy.Timer(rospy.Duration(0.5), self.send_TF)
		
	def init_Pin_List(self):
		self.Pin_List = [{'pose':[]},{'pose':[]}
							,{'pose':[]},{'pose':[]}]
		for i in range(4):
			for j in range(how_many_pins[i]):
				self.Pin_List[i]['pose'].append([])
	def init_TF_List(self):
		self.TF_List = [{'origin' : [], 'holes' : []},{'origin' : [],'holes' : []},{'origin' : [],'holes' : []}
						,{'origin' : [],'holes' : []},{'origin' : [],'holes' : []},{'origin' : [],'holes' : []}]

		for i in range(6):
			for j in range(holes_in_part[i]):
				self.TF_List[i]['holes'].append([])
	def init_GP_List(self):
		self.GP_List = [{'pose':[]},{'pose':[]},{'pose':[]},
						{'pose':[]},{'pose':[]},{'pose':[]}]

		for i in range(6):
			for j in range(grasp_in_part[i]):
				self.GP_List[i]['pose'].append([])
	def init_ASM_list(self):
		self.ASM_List = [{'part':[],'pin':[]},{'part':[],'pin':[]},{'part':[],'pin':[]},
							{'part':[],'pin':[]},{'part':[],'pin':[]},{'part':[],'pin':[]}]
	def init_attach_list(self):
		self.a_list = {'part':[],
						'pin':[]}

	def add_mesh(self, mesh_name, file_name, mesh_pose):
		_added = False
		mesh_pose.header.frame_id = "world"
		position_list = make_position_list(mesh_pose.pose.position)
		orientation_list = make_orientation_list(mesh_pose.pose.orientation)
		while not _added:	
			self.scene.add_mesh(mesh_name, mesh_pose, file_name, size=(1, 1, 1))
			if self.scene.get_objects([mesh_name]):
				_added = True
				break

		if mesh_name in part_name:
			part_num = part_name.index(mesh_name)
			self.TF_List[part_num]['origin'] = copy.deepcopy(mesh_pose.pose)

		print "[INFO] MESH NAME = '", mesh_name,"' ADDED"
	def add_pin(self, pin_type,pin_tag):
		#pin add at initial pose

		pin_pose = pin_arrayment[pin_type][pin_tag]

		Pin_name = pin_name[pin_type]+"-"+str(pin_tag+1)

		self.add_mesh(Pin_name,pin_file[pin_type],pin_pose)
		if pin_has_hole[pin_type]:
			pin_origin_pose = copy.deepcopy(pin_pose.pose)

			pin_hole_offset = HO.pin_hole_offset[pin_type]
			pin_hole_pose = self.get_TF_pose(pin_origin_pose,pin_hole_offset)

			pin_pose = copy.deepcopy({'pin':pin_origin_pose,'hole':pin_hole_pose})
		else:
			pin_pose = copy.deepcopy(pin_pose.pose)
		self.Pin_List[pin_type]['pose'][pin_tag] = pin_pose

	def set_parts(self,part_list =[0,1,2,3,4,5],pin_list = [0,1,2,3]):
		for p_num in part_list:
			self.add_mesh(part_name[p_num],part_file[p_num],part_ready_pose[p_num])
			self.set_part_TF(part_name[p_num])
		
		for pin_type in pin_list:
			for pin_tag in range(how_many_pins[pin_type]):
				self.add_pin(pin_type,pin_tag)
		self.part_add_flag = True
	def set_part_TF(self,mesh_name): # fill TF_list and GP_list
		part_num = part_name.index(mesh_name)
		if self.scene.get_objects([mesh_name]):
			if mesh_name in self.a_list['part']:
				self.TF_List[part_num]['origin']  = []
			else:
				mesh_pose = self.scene.get_objects([mesh_name])[mesh_name].mesh_poses[0]
				self.TF_List[part_num]['origin']  = copy.deepcopy(mesh_pose)

			
		num_of_holes = len(HO.hole_offset[part_num])
		for hole_num in range(num_of_holes):
			if self.TF_List[part_num]['origin'] == []:
				self.TF_List[part_num]['holes'][hole_num] = []
			else:
				hole_data = HO.hole_offset[part_num][hole_num]
				hole_pose = self.get_TF_pose(self.TF_List[part_num]['origin'],hole_data)
				self.TF_List[part_num]['holes'][hole_num] = copy.deepcopy(hole_pose)


		num_of_gp = len(GP.grasping_pose[part_num])
		for gp in range(num_of_gp):
			if self.TF_List[part_num]['origin'] == []:
				grasping_pose = []
			else:
				part_origin = self.TF_List[part_num]['origin']
				tf_data = GP.grasping_pose[part_num][gp]
				grasping_pose = self.get_TF_pose(part_origin,tf_data)
			self.GP_List[part_num]['pose'][gp] = copy.deepcopy(grasping_pose)
	def set_pin_TF(self,pin_type,pin_tag): # fill Pin_lsit #Pin_name = pin_name[pin_number]+"-"+str(pin_exist+1)
	
		target_pin_name = pin_name[pin_type]+"-"+str(pin_tag+1)
		if self.scene.get_objects([target_pin_name]):
			if target_pin_name in self.a_list['pin']:
				self.Pin_List[pin_type]['pose'][pin_tag] = []		
			else:
				target_pin_origin = self.scene.get_objects([target_pin_name])[target_pin_name].mesh_poses[0]
				TF_data = pin_TF_pose[pin_type]
				target_pin_TF = self.get_TF_pose(target_pin_origin,TF_data)

				if pin_has_hole[pin_type]:
					pin_origin_pose = copy.deepcopy(target_pin_TF)
					pin_hole_offset = HO.pin_hole_offset[pin_type]
					pin_hole_pose = self.get_TF_pose(pin_origin_pose,pin_hole_offset)
					pin_pose = copy.deepcopy({'pin':pin_origin_pose,'hole':pin_hole_pose})

				else:
					pin_pose = copy.deepcopy(target_pin_TF)
				self.Pin_List[pin_type]['pose'][pin_tag] = pin_pose

	def send_part_TF(self):
		for p_num in range(6):
			self.set_part_TF(part_name[p_num])
			if not self.TF_List[p_num]['origin'] == []: 
					selected_part = self.TF_List[p_num]
					position_list = make_position_list(selected_part['origin'].position)
					orientation_list = make_orientation_list(selected_part['origin'].orientation)

					self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), part_name[p_num], 'world')

					num_of_holes = len(HO.hole_offset[p_num])

					for h_num in range(num_of_holes):
						hole_name = "hole" + str(p_num+1) + "-" + str(h_num+1)

						position_list = make_position_list(selected_part['holes'][h_num].position)
						orientation_list = make_orientation_list(selected_part['holes'][h_num].orientation)

						self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), hole_name, 'world')
	def send_pin_TF(self):
		for pin_type in range(4):
			selected_pin = self.Pin_List[pin_type]

			for pins in range(how_many_pins[pin_type]):
				
				self.set_pin_TF(pin_type,pins)
				
				if not self.Pin_List[pin_type]['pose'][pins] == []:
					
					Pin_name = pin_name[pin_type]+"-"+str(pins+1)
					
					if not Pin_name in self.a_list['pin']:
						
						if pin_has_hole[pin_type] == True:
							Hole_name = Pin_name+"hole"
							#firstly pin
							position_list = make_position_list(selected_pin['pose'][pins]['pin'].position)
							orientation_list = make_orientation_list(selected_pin['pose'][pins]['pin'].orientation)
							self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), Pin_name, 'world')

							#secondary hole
							position_list = make_position_list(selected_pin['pose'][pins]['hole'].position)
							orientation_list = make_orientation_list(selected_pin['pose'][pins]['hole'].orientation)
							self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), Hole_name, 'world')
						else:
							position_list = make_position_list(selected_pin['pose'][pins].position)
							orientation_list = make_orientation_list(selected_pin['pose'][pins].orientation)
							self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), Pin_name, 'world')
	def send_GP_TF(self):
		for part_type in range(6):
			if not self.TF_List[part_type]['origin'] == []:
				num_of_gp = len(self.GP_List[part_type]['pose'])
				selected_part = self.GP_List[part_type]
				for g in range(num_of_gp):
					gp_name = part_name[part_type]+"-GRASP-"+str(g+1)
					position_list = make_position_list(selected_part['pose'][g].position)
					orientation_list = make_orientation_list(selected_part['pose'][g].orientation)
					self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), gp_name, 'world')
	def send_TF(self,dumb_Data):
		if self.part_add_flag == True:
			# print"[INFO] send_TF CALLED"
			self.send_part_TF()
			self.send_pin_TF()
			self.send_GP_TF()

	def get_TF_pose(self,origin_pose,TF_offset):
		#origin pose : pose of origin of part : mesh_pose.pose
		if origin_pose == []:
			hole_pose = []
		else:
			o_list_origin = copy.deepcopy(make_orientation_list(origin_pose.orientation))
			
			Rotation = quaternion_matrix(o_list_origin)

			axis_x = Rotation[0:3,0]
			axis_y = Rotation[0:3,1]
			axis_z = Rotation[0:3,2]

			hole_pose = geometry_msgs.msg.PoseStamped().pose
			hole_pose.position = copy.deepcopy(origin_pose.position)

			x_offset = axis_x[0]*TF_offset['trans'][0] + axis_y[0]*TF_offset['trans'][1] + axis_z[0]*TF_offset['trans'][2]
			y_offset = axis_x[1]*TF_offset['trans'][0] + axis_y[1]*TF_offset['trans'][1] + axis_z[1]*TF_offset['trans'][2]
			z_offset = axis_x[2]*TF_offset['trans'][0] + axis_y[2]*TF_offset['trans'][1] + axis_z[2]*TF_offset['trans'][2]

			hole_pose.position.x += x_offset
			hole_pose.position.y += y_offset
			hole_pose.position.z += z_offset

			rol_add = TF_offset['rot'][0]
			pit_add = TF_offset['rot'][1]
			yaw_add = TF_offset['rot'][2]

			Rotation_add = euler_matrix(rol_add,pit_add,yaw_add)
			Rotation_new = Rotation.dot(Rotation_add)

			(rol,pit,yaw) = euler_from_matrix(Rotation_new)
			o_list = quaternion_from_euler(rol,pit,yaw)
			hole_pose.orientation.x = o_list[0]
			hole_pose.orientation.y = o_list[1]
			hole_pose.orientation.z = o_list[2]
			hole_pose.orientation.w = o_list[3]
		return hole_pose
	def send_temporary_TF(self,pose_from_world,tf_name = "TEMP_POSE"):
		position_list = make_position_list(pose_from_world.position)
		orientation_list = make_orientation_list(pose_from_world.orientation)
		self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), tf_name, 'world')

	def assemble_part(self,target_part_name,assembling_part_name): # both names have to be string type
		if target_part_name in part_name and assembling_part_name in part_name:
			target_part_num = part_name.index(target_part_name)
			assembling_part_num = part_name.index(assembling_part_name)

			self.ASM_List[target_part_num]['part'].append(deepcopy(assembling_part_name))
			self.ASM_List[assembling_part_num]['part'].append(deepcopy(target_part_name))
		else:
			print "assemble_part : [WRONG_ATTACH_LIST]"
	def assemble_pin(self,target_part_name,assembling_pin_name): # both names have to be string type
		target_part_num = part_name.index(target_part_name)
		assembling_pin_num = pin_name.index(assembling_pin_name.split('-')[0])
		# assembling_pin_tag = assembling_pin_name.split('-')[1]-1
		self.ASM_List[target_part_num]['pin'].append(deepcopy(assembling_pin_name))
		self.ASM_List[target_part_num]['pin'].sort()
	def get_attach_list(self,target_mesh_name):
		attach_list = {'part':[],'pin':[]}
		attached_stack = []		# have been appended
		attaching_queue = []	# have to be appended
		attaching_queue.append(deepcopy(target_mesh_name))

		if target_mesh_name.split('-')[0] in pin_name:
			attach_list['pin'].append(deepcopy(target_mesh_name))

		elif target_mesh_name in part_name:

			while not attaching_queue == []:				
				target_part_num = part_name.index(attaching_queue[0])

				linked_part_list = self.ASM_List[target_part_num]['part']

				for linked_part_name in linked_part_list:
					if not linked_part_name in attached_stack:
						attaching_queue.append(deepcopy(linked_part_name))

				attach_list['part'].append(deepcopy(attaching_queue[0]))
				attached_stack.append(deepcopy(attaching_queue[0]))
				del attaching_queue[0]

				linked_pin_list = self.ASM_List[target_part_num]['pin']

				for target_pin_name in linked_pin_list:
					if not target_pin_name in attach_list['pin']:
						attach_list['pin'].append(deepcopy(target_pin_name))
		else:
			print "(get_attach_list) : WRONG_MESH_NAME"
		self.a_list = attach_list
		return attach_list

# def main():	
# 	rospy.init_node('TF_test', anonymous=True)
# 	TF = TF_Node()
# 	TF.set_parts()
# 	while True:
# 		try:
# 			print "PRES ENTER"
# 			raw_input()
# 		except rospy.ROSInterruptException:
# 			return
# 		except KeyboardInterrupt:
# 			return
# if __name__ == '__main__':
# 	main()