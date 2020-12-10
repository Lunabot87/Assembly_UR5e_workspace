from ur5e_inv_kin import UR5eInvKin
from utils.conversions import mat_to_tf
from utils.util import *
import tf.transformations as tf
import numpy as np

from numpy.linalg import inv

ur5e_ik = UR5eInvKin()

aa = ur5e_ik.fwd_kin([2.99847269058, -2.41846718411, 1.56093055407, -2.284062048 ,-1.40980798403, 3.14153337479])

print "aa : {0}".format(aa[:3,:3])

eef_trans, eef_mat = mat_to_tf(aa)


#rot_matrix = tf.quaternion_matrix(eef_mat.tolist())

rot_zaxis = aa[:3,2]

print "rot_zaxis : {0}".format(rot_zaxis)

#print rot_matrix[:3,:3]

# z_matrix = tf.translation_matrix([0, 1, 0])

z_axis_global = [0,0,1]

y_axis_global = [0,1,0]

x_axis_global = [1,0,0]

print "z_axis_global : {0}".format(z_axis_global)

axis_z = np.cross(z_axis_global, rot_zaxis)

axis_y = np.cross(y_axis_global, rot_zaxis)

axis_x = np.cross(x_axis_global, rot_zaxis)

print "axis z: {0}".format(axis_z)

print "axis y: {0}".format(axis_y)

print "axis x: {0}".format(axis_x)

print zAxis_difference(x_axis_global, rot_zaxis)

# print np.allclose(z_axis_global, rot_zaxis)



# print np.dot(z_matrix[:3,:3], rot_matrix[:3,:3])

# q = tf.quaternion_about_axis(0.123, (0,-1,0))

# print np.allclose(q, eef_mat.tolist())