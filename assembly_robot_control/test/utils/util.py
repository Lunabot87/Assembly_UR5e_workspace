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

def get_constraints_from_dict(dictType, ref_name):
    consts = {}

    robot = dictType["robot"]
    robotName = robot["@name"]
    robotCM = get_CM_from_link(robot["link"][0])
    robotMass = float(robot["link"][0]["inertial"]["mass"]["@value"])
    jointList = make_it_list(robot["joint"])
    linkList = make_it_list(robot["link"][1:])

    for joint, link in zip(jointList, linkList):
        name_split = link["@name"].split(" ")
        constType = str(name_split[1])
        constFeature = str(name_split[2])
        idx = int(name_split[3])

        constName = "{}-{}_{}".format(ref_name, constType, idx)

        origin_xyz, origin_rpy = get_xyz_rpy_from_joint(joint)
        origin_quat = tf.quaternion_from_euler(origin_rpy[0], origin_rpy[1], origin_rpy[2])

        if constType == "hole" or constType == "pin":
            const = {}
            length = float(link["visual"]["geometry"]["cylinder"]["@length"])
            radius = float(link["visual"]["geometry"]["cylinder"]["@radius"])
            endCoordinate = get_translated_origin(origin_xyz, origin_quat, [0, 0, length/2])
            entryCoordinate = get_translated_origin(origin_xyz, origin_quat, [0, 0, -length/2])
            const["endCoordinate"] = endCoordinate
            const["entryCoordinate"] = entryCoordinate
            const["name"] = constName
            const["feature"] = constFeature
            const["satisfied"] = False
            const["length"] = length
            const["radius"] = radius
            const["type"] = constType
            const["contactWith"] = None
            if constFeature == "insert":
                spare_const = copy.deepcopy(const)
                spare_rpy = origin_rpy + np.array([m.pi, 0, 0])
                spare_quat = tf.quaternion_from_euler(spare_rpy[0], spare_rpy[1], spare_rpy[2])
                spare_endCoordinate = get_translated_origin(origin_xyz, spare_quat, [0, 0, length/2])
                spare_entryCoordinate = get_translated_origin(origin_xyz, spare_quat, [0, 0, -length/2])
                spare_const["endCoordinate"] = spare_endCoordinate
                spare_const["entryCoordinate"] = spare_entryCoordinate
                consts[constName+"_spare"] = spare_const
            consts[constName] = const
        else:
            # rospy.logerr("Given constType('{}') is not valid".format(constType))
            return TypeError

    return consts, robotCM, robotMass