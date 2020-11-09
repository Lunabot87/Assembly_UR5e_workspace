# import rospy
import numpy as np
import math as m
import tf.transformations as tf
import copy

def get_CM_from_link(link):
    cm = link["inertial"]["origin"]["@xyz"].split(" ")
    return np.array(map(float, cm))

def make_it_list(target):
    if type(target) is not list:
        return [target]
    else:
        return target

def get_xyz_rpy_from_joint(joint):
    xyz = joint["origin"]["@xyz"].split(" ")
    rpy = joint["origin"]["@rpy"].split(" ")
    return np.array(map(float, xyz)), np.array(map(float, rpy))

def get_tf_matrix(translation, quat_rotation):
    trans_matrix = tf.translation_matrix(translation)
    rot_matrix = tf.quaternion_matrix(quat_rotation)
    tf_matrix = tf.concatenate_matrices(trans_matrix, rot_matrix)
    return tf_matrix

def get_translated_origin(origin_xyz, origin_quat, translation):
    target_translation = tf.translation_matrix(translation)
    tf_matrix = get_tf_matrix(origin_xyz, origin_quat)
    translated_origin = tf.concatenate_matrices(tf_matrix, target_translation)
    return translated_origin

def get_transformed_zAxis(quat_rotation):
    z_matrix = tf.translation_matrix([0, 0, 1])
    rot_matrix = tf.quaternion_matrix(quat_rotation)
    return tf.concatenate_matrices(rot_matrix, z_matrix)[:3, 3]

def zAxis_difference(z1, z2):
    return abs(np.dot(z1, z2))

    