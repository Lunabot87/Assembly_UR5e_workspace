#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import time
import copy
import random

from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from math import pi

from geometry_msgs.msg import *
from tf.transformations import *
from tf import *

import hole_offsets as HO 		# locations of holes of each part
from part_initial_pose import*	# set initial poses of parts
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
		self.init_TFlist()
		self.init_PinList()

		rospy.Timer(rospy.Duration(1), self.send_TF)

	def init_PinList(self):
		self.PinList = [{'name':[],'pose':[]},{'name':[],'pose':[]}
								,{'name':[],'pose':[]},{'name':[],'pose':[]}]
	
	def init_TFlist(self):
		self.TFlist = [{'origin' : [], 'holes' : []},{'origin' : [],'holes' : []},{'origin' : [],'holes' : []}
						,{'origin' : [],'holes' : []},{'origin' : [],'holes' : []},{'origin' : [],'holes' : []}]

	def make_orientation_list(self,orientation):
		o_list = [orientation.x, orientation.y, orientation.z, orientation.w]
		return o_list

	def make_position_list(self,position):
		p_list = [position.x,position.y,position.z]
		return p_list

	def add_mesh(self, mesh_name, file_name, mesh_pose):
		mesh_pose.header.frame_id = "world"
		self.scene.add_mesh(mesh_name, mesh_pose, file_name, size=(1, 1, 1))
		position_list = self.make_position_list(mesh_pose.pose.position)
		orientation_list = self.make_orientation_list(mesh_pose.pose.orientation)
		self.scene.add_mesh(mesh_name, mesh_pose, file_name, size=(1, 1, 1))
		self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), mesh_name, 'world')
		print "[INFO] MESH NAME = '", mesh_name,"' ADDED"

	def add_pin(self, pin_number):
		#pin_number is its index in the list
		pin_exist = len(self.PinList[pin_number]['pose'])
		pin2pin_dist = pin_diameter[pin_number] * 3
		pin_pose = copy.deepcopy(pin_pose_base[pin_number])
		pin_pose.pose.position.y += pin2pin_dist * pin_exist * pin_side[pin_number]

		Pin_name = pin_name[pin_number]+"-"+str(pin_exist)

		self.add_mesh(Pin_name,pin_file[pin_number],pin_pose)
		if pin_has_hole[pin_number]:
			pin_origin_pose = copy.deepcopy(pin_pose.pose)
			pin_hole_pose = geometry_msgs.msg.PoseStamped().pose
			pin_hole_pose.orientation.w = 1
			# pin_hole_pose hasn't set yet.
			pin_pose = copy.deepcopy({'pin':pin_origin_pose,'hole':pin_hole_pose})
		else:
			pin_pose = copy.deepcopy(pin_pose.pose)
		self.PinList[pin_number]['pose'].append(pin_pose)

	def send_part_TF(self):
		for p_num in range(6):
			selected_part = self.TFlist[p_num]
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
		for pintype in range(4):
			_pin_exist = len(self.PinList[pintype]['pose'])
			selected_pin = self.PinList[pintype]

			for pins in range(_pin_exist):
				if pin_has_hole[pintype] == True:
					Pin_name = pin_name[pintype]+"-"+str(pins)
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
					Pin_name = pin_name[pintype]+"-"+str(pins)
					position_list = self.make_position_list(selected_pin['pose'][pins].position)
					orientation_list = self.make_orientation_list(selected_pin['pose'][pins].orientation)
					self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), Pin_name, 'world')

	def send_TF(self,dumb_Data):
		if self.mesh_added_flag == True:
			# print"[INFO] send_TF CALLED"
			self.send_part_TF()
			self.send_pin_TF()

	def set_parts(self):
		for p_num in range(6):
			time.sleep(0.3)
			self.add_mesh(part_name[p_num],part_file[p_num],part_pose[p_num])
			self.get_part_pose(part_name[p_num])
		
		for i in range(4):
			for j in range(random.randrange(1,10)):
				self.add_pin(i)


		self.mesh_added_flag = True

	def set_hole_pose(self,origin_pose,part_num,hole_num):
		#origin pose : pose of origin of part : mesh_pose.pose

		o_list_origin = copy.deepcopy(self.make_orientation_list(origin_pose.orientation))
		
		Rotation = quaternion_matrix(o_list_origin)

		axis_x = Rotation[0:3,0]
		axis_y = Rotation[0:3,1]
		axis_z = Rotation[0:3,2]

		hole_pose = geometry_msgs.msg.PoseStamped().pose
		hole_pose.position = copy.deepcopy(origin_pose.position)

		_hole_offset = HO.hole_offset[part_num][hole_num]
	
		x_offset = axis_x[0]*_hole_offset['trans'][0] + axis_y[0]*_hole_offset['trans'][1] + axis_z[0]*_hole_offset['trans'][2]
		y_offset = axis_x[1]*_hole_offset['trans'][0] + axis_y[1]*_hole_offset['trans'][1] + axis_z[1]*_hole_offset['trans'][2]
		z_offset = axis_x[2]*_hole_offset['trans'][0] + axis_y[2]*_hole_offset['trans'][1] + axis_z[2]*_hole_offset['trans'][2]

		hole_pose.position.x += x_offset
		hole_pose.position.y += y_offset
		hole_pose.position.z += z_offset


		(rol,pit,yaw) = euler_from_quaternion(o_list_origin)
		rol += _hole_offset['rot'][0]
		pit += _hole_offset['rot'][1]
		yaw += _hole_offset['rot'][2]
		o_list = quaternion_from_euler(rol,pit,yaw)
		hole_pose.orientation.x = o_list[0]
		hole_pose.orientation.y = o_list[1]
		hole_pose.orientation.z = o_list[2]
		hole_pose.orientation.w = o_list[3]

		self.TFlist[part_num]['holes'].append(copy.deepcopy(hole_pose))

	def get_part_pose(self,mesh_name):
		part_num = part_name.index(mesh_name)
		mesh_pose = geometry_msgs.msg.PoseStamped()
		mesh_pose.header = self.scene.get_objects([mesh_name])[mesh_name].header
		mesh_pose.pose = self.scene.get_objects([mesh_name])[mesh_name].mesh_poses[0]

		self.TFlist[part_num]['origin']  = copy.deepcopy(mesh_pose.pose)
		num_of_holes = len(HO.hole_offset[part_num])
		
		for hm in range(num_of_holes):
			self.set_hole_pose(mesh_pose.pose,part_num,hm)

	def get_pin_pose(self,pin_name): # 내일 만들자
		for pintype in range(4):
			_pin_exist = len(self.PinList[pintype]['pose'])
			selected_pin = self.PinList[pintype]

			for pins in range(_pin_exist):
				if pin_has_hole[pintype] == True:
					
				else:


	def update_tf(self):
		self.mesh_added_flag = False
		self.init_TFlist()
		for p_num in range(6):
			time.sleep(0.3)
			self.get_part_pose(part_name[p_num])


		self.mesh_added_flag = True



def init():
	TF = TF_Node()
	TF.set_parts()

	print "Press enter"
	raw_input()
	while True:
		TF.update_tf()

		print "Press ENTER"
		raw_input()



if __name__ == '__main__':
	init()