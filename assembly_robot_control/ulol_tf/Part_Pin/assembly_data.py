# parts' offsetations based on part6
import hole_offsets as HO
import numpy as np
import copy
from tf.transformations import *
from math import pi
def add_2transrpy(cur_transrpy,transrpy_offset):
	pt = cur_transrpy['trans']
	prot = cur_transrpy['rot']
	pT = euler_matrix(prot[0],prot[1],prot[2])
	pT[0:3,3] = pt

	ot = transrpy_offset['trans']
	orot = transrpy_offset['rot']
	oT = euler_matrix(orot[0],orot[1],orot[2])
	oT[0:3,3] = ot

	newT = np.dot(pT,oT)

	new_trans = newT[0:3,3]
	new_rpy = euler_from_matrix(newT)

	new_dict = {'trans':new_trans,'rot':new_rpy}
	return new_dict

assembly_data_list = []

chair_part1_offset = {'trans':[],'rot':[]}
chair_part2_offset = {'trans':[-0.005,0.016,-0.175],'rot':[0,pi/2,0]}	# hole6-3
chair_part3_offset = {'trans':[0.005,0.016,-0.142],'rot':[0,pi/2,pi]}			# hole6-6
chair_part4_offset = {'trans':[0.005,0.016,-0.142],'rot':[0,pi/2,0]}			# hole6-8
chair_part5_offset = {'trans':[],'rot':[]}

chair_part2_assemble_offset = add_2transrpy(HO.P6hole3,chair_part2_offset)
chair_part3_assemble_offset = add_2transrpy(HO.P6hole6,chair_part3_offset)
chair_part4_assemble_offset = add_2transrpy(HO.P6hole8,chair_part4_offset)

