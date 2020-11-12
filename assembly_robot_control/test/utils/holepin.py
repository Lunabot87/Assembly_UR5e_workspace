#-*- coding:utf-8 -*-

from const import *
from conversions import *

import util
import numpy as np 

pi = 3.1415

def get_asm_pose_by_HolePin(ref_coors_raw, move_coors_raw): # mat
    target_tr = np.array([0, 0, 0])
    target_quat = np.array([0, 0, 0, 1])
    ref_axis = np.array([0, 0, -1])
    criterion = []
    move_coors = []
    ref_coors = []
    success = False

    for i, j in zip(ref_coors_raw, move_coors_raw):
        ref_coors.append(tf_to_mat(i[:3], i[3:]))
        move_coors.append(tf_to_mat(j[:3], j[3:]))


    # ref_coors = [tf_to_mat([0.369042, 0.407433,-0.050], tf.transformations.quaternion_from_euler(0,0,0)), \
    # tf_to_mat([0.367368, 0.375477,-0.050], tf.transformations.quaternion_from_euler(0,0,0))] 

    # parent P6hole3={'trans':[0.369042, 0.407433,-0.050],'rot':[0,0,0]} 
    # P6hole4={'trans':[0.367368, 0.375477,-0.050],'rot':[0,0,0]}

    ref_quats = [tf.transformations.quaternion_from_matrix(coor) for coor in ref_coors]

    # print('ref_quats : {0}'.format(ref_quats)) #회전값 출력

    ref_trs = [tf.transformations.translation_from_matrix(coor) for coor in ref_coors]

    # print('ref_trs : {0}'.format(ref_trs)) #좌표값 출력 

    ref_zs = [util.get_transformed_zAxis(quat) for quat in ref_quats]

    # print('ref_zs : {0}'.format(ref_zs)) #z축 기준으로 얼마나 회전했는지 출력 

    ref_z_diffs = [util.zAxis_difference(ref_zs[0], zAxis) for zAxis in ref_zs]

    # print('ref_z_diffs : {0}'.format(ref_z_diffs))

    ref_tr_diffs = [tr - ref_trs[0] for tr in ref_trs]

    # print('after ref_z_diffs : {0}'.format(ref_z_diffs))

    # move_coors = [const[cri+"Coordinate"] for (const, cri) \
    #     in zip(move_consts, criterion)]

    # move_coors = [tf_to_mat([-0.125-0.0065, 0.0160, 0.01], tf.transformations.quaternion_from_euler(0,pi/2,0)), \
    # tf_to_mat([-0.125-0.0065,-0.0160, 0.01], tf.transformations.quaternion_from_euler(0,pi/2,0))] 

    # child P3hole3={'trans':[-0.125-0.0065, 0.0160, 0.01],'rot':[0,pi/2,0]}
					# P3hole4={'trans':[-0.125-0.0065,-0.0160, 0.01],'rot':[0,pi/2,0]}

    move_quats = [tf.transformations.quaternion_from_matrix(coor) for coor in move_coors]
    # print('move_quats : {0}'.format(move_quats))
    move_trs = [tf.transformations.translation_from_matrix(coor) for coor in move_coors]
    # print('move_trs : {0}'.format(move_trs))
    move_zs = [util.get_transformed_zAxis(quat) for quat in move_quats]
    # print('move_zs : {0}'.format(move_zs))
    move_z_diffs = [util.zAxis_difference(move_zs[0], zAxis) for zAxis in move_zs]
    # print('move_z_diffs : {0}'.format(move_z_diffs))
    move_tr_diffs = [tr - move_trs[0] for tr in move_trs]

    is_same_z_diffs = [np.allclose(ref_z_diff, move_z_diff, rtol=0.005, atol=0.005) \
        for (ref_z_diff, move_z_diff) in zip(ref_z_diffs, move_z_diffs)]
    if not is_same_z_diffs:
        print("ref_z_diffs : {0}".format(ref_z_diffs))
        print("move_z_diffs : {0}".format(move_z_diffs))
        print("is_same_z_diffs : {0}".format(is_same_z_diffs))
        rospy.logwarn("Target hole and pin's direction is not proper for assembly!")
        pass
    else:
        if len(ref_coors) > 1:
            move_quat_inv1 = tf.transformations.quaternion_inverse(move_quats[0])
            temp_quat = tf.transformations.quaternion_multiply(ref_quats[0], move_quat_inv1)

            ref_axis = ref_zs[0]

            move2_tr_from_ref1_temp = \
                util.get_translated_origin([0,0,0], temp_quat, move_tr_diffs[1])[:3, 3]
            move2_tr_from_ref1_temp_re = \
                util.get_translated_origin([0,0,0], tf.transformations.quaternion_inverse(ref_quats[0]), move2_tr_from_ref1_temp)[:3, 3]
            ref2_tr_from_ref1_temp = \
                util.get_translated_origin([0,0,0], tf.transformations.quaternion_inverse(ref_quats[0]), ref_tr_diffs[1])[:3, 3]
            
            grad_move2_from_ref1_temp = \
                np.arctan2(move2_tr_from_ref1_temp_re[1], move2_tr_from_ref1_temp_re[0])
            grad_ref2_from_ref1_temp = \
                np.arctan2(ref2_tr_from_ref1_temp[1], ref2_tr_from_ref1_temp[0])
            
            temp_grad1 = tf.transformations.quaternion_about_axis(grad_move2_from_ref1_temp, ref_axis)
            temp_grad1_inv = tf.transformations.quaternion_inverse(temp_grad1)
            temp_grad2 = tf.transformations.quaternion_about_axis(grad_ref2_from_ref1_temp, ref_axis)

            target_quat = tf.transformations.quaternion_multiply(temp_grad1_inv, temp_quat)
            target_quat = tf.transformations.quaternion_multiply(temp_grad2, target_quat)
            target_tr = util.get_translated_origin(ref_trs[0], target_quat, -move_trs[0])[:3, 3]
        else:
            ref_axis = ref_zs[0]

            move_quat_inv = tf.transformations.quaternion_inverse(move_quats[0])
            target_quat = tf.transformations.quaternion_multiply(ref_quats[0], move_quat_inv)
            target_tr = util.get_translated_origin(ref_trs[0], target_quat, -move_trs[0])[:3, 3]
            
    return target_tr, target_quat, ref_axis


    
def get_min_vector_from_ref(self, zs, trs, tr_diffs):
        projs = [np.dot(zAxis, tr_diff)*zAxis for (zAxis, tr_diff) \
            in zip(zs, tr_diffs)]
        tr_min = [tr - (trs[0]+proj) for (tr, proj) in zip(trs, projs)]
        return tr_min


if __name__ == '__main__':
   print(get_asm_pose_by_HolePin(1,1))