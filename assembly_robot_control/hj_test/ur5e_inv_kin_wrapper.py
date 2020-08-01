#!/usr/bin/env python
import math as m
import tf

import rospy
import std_msgs.msg
import moveit_msgs.msg
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity

from ur5e_inv_kin import UR5eInvKin
from utils.const import *
from utils.conversions import *

class UR5eInvKinForTF():
  def __init__(self, group_name, eef_offset):
    # ur5e kinematics
    self.ur5e_ik = UR5eInvKin(eef_offset)

    # Publisher
    self.robot_state_pub = rospy.Publisher('/inv_kin_sol', moveit_msgs.msg.DisplayRobotState, queue_size=1)
    
    # Subscriber
    self.aco_sub = rospy.Subscriber('/attached_collision_object', moveit_msgs.msg.AttachedCollisionObject, self._aco_cb)

    # Service
    self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    self.sv_srv.wait_for_service()
    print "service is available"
    
    # variables
    self.group_name = group_name
    self.w = [1.0, 1.0, 1.0, 0.5, 0.5, 0.2]
    self.aco = None

  ##############################################################################
  ##############################################################################
  def _get_diff_idx(self, inv_sol, cur_joint):
    # diffs = []
    order_diff = []
    for i in range(8):
      diff = 0.0
      for j in range(6): 
        diff += self.w[j] * (inv_sol[j][i] - cur_joint[j])**2 
      # diffs.append(diff)
      order_diff.append({'index': i, 'diff': float(diff)})
    def sortKey(e):
      return e['diff']

    order_diff.sort(key=sortKey)

    diff_order= []
    for i in range(8):
      diff_order.append(order_diff[i]['index'])
    
    return diff_order

  def _apply_joint_limit(self, inv_sol, cur_joint):
    '''
    input : [-180, 180]
    output : [-360, 360]
    '''
    for j in range(6):
      for i in range(8):
        cur = inv_sol[j, i]
        if cur < 0:
          tmp = cur + 2*m.pi
          if abs(tmp-cur_joint[j]) < abs(cur-cur_joint[j]):
            inv_sol[j, i] = tmp
        elif cur > 0:
          tmp = cur - 2*m.pi
          if abs(tmp-cur_joint[j]) < abs(cur-cur_joint[j]):
            inv_sol[j, i] = tmp
        else:
          tmp1 = cur + 2*m.pi
          tmp2 = cur - 2*m.pi
          if abs(tmp1-cur_joint[j]) < abs(cur-cur_joint[j]):
            if abs(tmp2-cur_joint[j]) < abs(tmp1-cur_joint[j]):
              inv_sol[j, i] = tmp2
            else:
              inv_sol[j, i] = tmp1

    return inv_sol

  def _aco_cb(self, msg):
    self.aco = msg

  def _get_state_validity(self, joint):
    # print(joint)
    robot_state = moveit_msgs.msg.RobotState()
    if self.group_name == 'rob1_arm':
      robot_state.joint_state.name = ROB1_JOINTS
    elif self.group_name == 'rob2_arm':
      robot_state.joint_state.name = ROB2_JOINTS
    robot_state.joint_state.position = joint
    
    if self.aco is not None:
      robot_state.attached_collision_objects = [self.aco]

    gsvr = GetStateValidityRequest()
    gsvr.robot_state = robot_state

    result = self.sv_srv.call(gsvr)
    rv = result.valid
    rc = result.contacts

    # print("+"*70)
    # print('state validity result')
    # print('valid = {}'.format(rv))
    # for i in range(len(rc)):
    #   print('contact {}: 1-{}, 2-{}'.format(i, rc[i].contact_body_1, rc[i].contact_body_2))

    return rv

  def _set_link_colors(self, r, g, b, a):
    if self.group_name == 'rob1_arm':
      LINKS_ENABLE = ROB1_ARM_LINK + ROB1_GRIPPER_LINK
      LINKS_DISABLE = ROB2_ARM_LINK + ROB2_GRIPPER_LINK
    elif self.group_name == 'rob2_arm':
      LINKS_ENABLE = ROB2_ARM_LINK + ROB2_GRIPPER_LINK
      LINKS_DISABLE = ROB1_ARM_LINK + ROB1_GRIPPER_LINK
    

    link_colors = []

    for i in range(len(LINKS_ENABLE)):
      color = moveit_msgs.msg.ObjectColor()
      color.id = LINKS_ENABLE[i]
      color.color.r = r
      color.color.g = g
      color.color.b = b
      color.color.a = a
      link_colors.append(color)
    
    for i in range(len(LINKS_DISABLE)):
      color = moveit_msgs.msg.ObjectColor()
      color.id = LINKS_DISABLE[i]
      color.color.a = 0.0
      link_colors.append(color)

    return link_colors

  ##############################################################################
  ##############################################################################
  def set_eef_offset(self, eef_offset):
    self.ur5e_ik = UR5eInvKin(eef_offset)
  
  def publish_state(self, selected_inv_sol, valid=None):
    '''
    publishes selected sol of robot state
      1) check state validity
      2) if valid, publish green robot
          else, publish red robot
    '''    
    if valid is None:
      valid = self._get_state_validity(selected_inv_sol)
    
    if valid:
      link_colors = self._set_link_colors(0, 20, 0, 0.7)
    else:
      link_colors = self._set_link_colors(20, 0, 0, 0.7)

    robot_state = moveit_msgs.msg.DisplayRobotState()
    robot_state.state.joint_state.header.frame_id = "world"
    if self.group_name == 'rob1_arm':
      robot_state.state.joint_state.name = ROB1_JOINTS
    elif self.group_name == 'rob2_arm':
      robot_state.state.joint_state.name = ROB2_JOINTS
    robot_state.state.joint_state.position = selected_inv_sol
    robot_state.state.multi_dof_joint_state.header.frame_id = "world"
    if self.aco is not None:
      robot_state.state.attached_collision_objects = [self.aco]
    robot_state.highlight_links = link_colors
    robot_state.state.is_diff = True

    self.robot_state_pub.publish(robot_state)

    return valid, selected_inv_sol

  def inv_kin(self, trans, rot):
    pose_mat = tf_to_mat(trans, rot)
    inv_sol = self.ur5e_ik.inv_kin(pose_mat)

    return inv_sol

  def inv_kin_limited(self, trans, rot, cur_joint):
    inv_sol = self.inv_kin(trans, rot)
    inv_sol_limited = self._apply_joint_limit(inv_sol, cur_joint)
    
    return inv_sol_limited

  def inv_kin_full(self, trans, rot, cur_joint):
    '''
    inv_sol_full : list of dictionary
    '''
    inv_sol = self.inv_kin(trans, rot)
    # inv_sol = self.inv_kin_limited(trans, rot, cur_joint)
    # sorted_idx, sorted_diff = self._get_diff_idx(inv_sol, cur_joint)

    inv_sol_full = []
    wrist_down = [0, 1, 6, 7]
    for i in range(8):
      diff = 0.0
      if i in wrist_down:
        diff += (m.pi)**2
      for j in range(6): 
        diff += self.w[j] * (inv_sol[j][i] - cur_joint[j])**2 
      sol = inv_sol[:, i]
      valid = self._get_state_validity(sol)
      inv_sol_full.append({'idx':i, 'valid': valid, 'diff': diff, 'inv_sol': sol})

    return inv_sol_full

  def inv_kin_full_sorted(self, trans, rot, cur_joint):
    inv_sol_full = self.inv_kin_full(trans, rot, cur_joint)
    inv_sol_full_sorted = sorted(inv_sol_full, key = lambda x: x['diff'])

    return inv_sol_full_sorted

  def print_inv_sol(self, list_of_dict):
    len_of_list = len(list_of_dict)
    num_of_dict = len(list_of_dict[0])

    print "="*100
    for j in range(num_of_dict):
      print(list_of_dict[0].keys()[j]),"\t\t",
    print ""
    
    for i in range(len_of_list):
      for j in range(num_of_dict):
        print(list_of_dict[i].items()[j][1]),"\t",
      print ""

def main():
  rospy.init_node('ur5_inv_kin_wrapper')  
  ur5e = UR5eInvKinForTF()
  listener = tf.TransformListener() 
  (trans, rot) = listener.lookupTransform('/rob1_real_base_link', '/rob1_real_ee_link', rospy.Time(0))
  print ur5e.inv_kin(trans, rot)

if __name__ == '__main__':
  main()