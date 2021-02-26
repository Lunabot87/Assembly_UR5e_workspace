#!/usr/bin/env python
import tf
import rospy
import time
import math as m
import copy 
import geometry_msgs.msg
import moveit_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander

from ur5e_inv_kin_wrapper import UR5eInvKinForTF
# from Parts_hj import Parts
from utils.const import *
from utils.conversions import *

def list_str(some_list, some_key):
  msg = ""
  try:
    for i in range(len(some_list)):
      msg += "{}: {:.3f} ".format(some_key[i], some_list[i])
  except:
    print "failed to make list look pretty!"

  return msg

class MoveGroupCommanderWrapper(MoveGroupCommander):
  def __init__(self, name, _eef_link=None):
    # super(MoveGroupCommanderWrapper, self).__init__(group_name)
    MoveGroupCommander.__init__(self, name)
    
    # setup for move group
    self.set_planning_time(3)
    self.set_num_planning_attempts(5)
    self.set_max_velocity_scaling_factor(1)
    self.set_max_acceleration_scaling_factor(1)

    if _eef_link is None:
      self.eef_link = 'tcp_gripper_closed'
      _eef_link = name[:5] + self.eef_link
    else:
      self.eef_link = _eef_link[5:]    
    MoveGroupCommander.set_end_effector_link(self, _eef_link)
    
    # setup for ur5e invkin
    eef_offset = TCP_OFFSET[self.eef_link]
    self.ur5e = UR5eInvKinForTF(name, eef_offset)
    print"eef_link: {}, eef_offset: {}".format(self.eef_link, eef_offset)


    # setup for scene
    self.scene = PlanningSceneInterface()
    self.robot = RobotCommander()

  def get_name(self):
    '''
    how to handle method overloading
    '''
    print "My name is {}".format(self.__class__.__name__)
    # return super(MoveGroupCommanderWrapper, self).get_name()
    return MoveGroupCommander.get_name(self)

  #####
  def attach(self, rob, part, file_name):
    touch_links = self.robot.get_link_names(group=rob+'_hand')
    self.scene.attach_mesh(rob+"_"+self.eef_link, part, file_name, touch_links=touch_links)

  def dettach(self, rob, part):
    self.scene.remove_attached_object(rob+"_"+self.eef_link, part)

  #####

  def _list_to_js(self, joint_list):
    js = JointState()
    js.name = self.get_active_joints()
    js.position = joint_list
    return js

  def set_end_effector_link(self, _eef_link):
    MoveGroupCommander.set_end_effector_link(self, _eef_link)
    
    self.eef_link = _eef_link[5:]
    eef_offset = TCP_OFFSET[self.eef_link]
    self.ur5e.set_eef_offset(eef_offset)

  def _joint_verification(self, joints):
    eef_mat = self.ur5e.ur5e_ik.fwd_kin(joints)
    rot_zaxis = eef_mat[:3,2]

    z_axis = [0,0,1]
    y_axis = [0,1,0]
    x_axis = [1,0,0]

    return [np.dot(x_axis, rot_zaxis), np.dot(y_axis, rot_zaxis), np.dot(z_axis, rot_zaxis)]

  def _get_best_ik_plan(self, trans, rot, c):
    '''
    [output]
    val1: if best solution with no collision and successful plan exists,
            return idx of best solution
          else,
            return -1
    val2: if value1 >-1,
            return successful trajectory
          else,
            return -1
    '''

    cur_joint = self.get_current_joint_values()
    
    print "get_best_ik"

    inv_sol = self.ur5e.inv_kin_full_sorted(trans, rot, cur_joint, c)

    # print inv_sol
    self.ur5e.print_inv_sol(inv_sol)
    
    #print "="*100
    #print "current q: ", cur_joint

    # for i in range(8):
    #   # if inv_sol[i]['valid']:
    #   selected_q = (inv_sol[i]['inv_sol'])
    #   print "selected q: ", selected_q

    #   self.ur5e.publish_state(selected_q, True)
    #   # print self.scene.getGlobalLinkTransform(self.eef_link)
    #   print "next pass"
    #   raw_input()

    for i in range(8):
      if inv_sol[i]['valid']:
        selected_q = (inv_sol[i]['inv_sol'])
        

        self.ur5e.publish_state(selected_q, True)
        # ------------------updated version
        # (success, traj, b, err) = self.plan(self._list_to_js(selected_q))        
        # if success:
        #   print "plan success: {}".format(b)
        #   user_choice = raw_input("--> press [y/n(wrong ik)]")  
        #   if user_choice == 'y':
        #     return inv_sol[i]['idx'], traj
        #   else:
        #     continue        
        # else:
        #   print "plan error:", b, err
        # ------------------updated version

        axis = self._joint_verification(selected_q)
        print "axis : {0}".format(axis)
        abs_axis = map(abs, axis)
        if abs_axis.index(max(abs_axis)) == 1 and m.ceil(axis[1]) == 1:
          continue
        elif selected_q[2] < 0 and selected_q[0] > 0:
          continue
        elif selected_q[2] > 0 and selected_q[0] < 0:
          continue
        else:
          traj = self.plan(self._list_to_js(selected_q))
          print "selected q: ", selected_q
        #user_choice = raw_input("--> press [y/n(wrong ik)]") 
        user_choice = 'y'
        if user_choice == 'y':
          return inv_sol[i]['idx'], traj
        else:
          continue 



    return -1, -1

  def _get_selected_ik_plan(self, trans, rot, idx, c=False):
    '''
    [output]
    val1: if selected idx has no collision and successful plan,
            return selected idx
          else,
            return -1
    val2: if value1 >-1,
            return successful trajectory
          else,
            return -1
    '''
    cur_joint = self.get_current_joint_values()
    inv_sol = self.ur5e.inv_kin_full(trans, rot, cur_joint, c)
    self.ur5e.print_inv_sol(inv_sol)
    
    # print "="*100
    # print "current q: ", cur_joint
    
    if inv_sol[idx]['valid']:
      selected_q = (inv_sol[idx]['inv_sol'])
      # print "selected q: ", selected_q
      
      self.ur5e.publish_state(selected_q, True)
      # ------------------updated version
      # (success, traj, b, err) = self.plan(self._list_to_js(selected_q))
      # if success:
      #   print "plan success: {}".format(b)
      #   return inv_sol[idx]['idx'], traj
      # else:
      #   print "plan error: {}, {}".format(b, err)
      # ------------------updated version
      traj = self.plan(self._list_to_js(selected_q))
      return inv_sol[idx]['idx'], traj
    
    else:
      print "selected solution is invalid"

    
    return -1, -1

  def go_to_initial_pose(self):
    joint_goal = [-m.pi/2, -m.pi/2,-m.pi/2,-m.pi/2, m.pi/2, m.pi]
    
    self.go(joint_goal, wait=True)
    # self.stop()

  def go_to_pose_goal(self, trans, rot, idx=None, c=False):
    '''
    [output]
    if go_to_pose_goal succeeded,
      return used ik idx
    else,
      return -1
    '''

    print "go_to_pose_goal"
    if idx is None:
      (s_idx, traj) = self._get_best_ik_plan(trans, rot)
    else:
      (s_idx, traj) = self._get_selected_ik_plan(trans, rot, idx, c)

    if s_idx >= 0:
      self.execute(traj, wait=True)
      self.stop()
    
    return s_idx

  def go_to_part_goal(self, trans, rot, idx=None, c=False):
    '''
    [output]
    if go_to_pose_goal succeeded,
      return used ik idx
    else,
      return -1
    '''
    if idx is None:
      (s_idx, traj) = self._get_best_ik_plan(trans, rot)
    else:
      (s_idx, traj) = self._get_selected_ik_plan(trans, rot, idx, c)

    if s_idx >= 0:
      self.execute(traj, wait=True)
      self.stop()
    else:
      return -1
    
    return s_idx

  def go_linear_to_pose_goal(self, trans, rot, idx):
    '''
    [output]
    if go_linear_to_pose_goal succeeded,
      return used ik idx
    else,
      return -1
    '''
    cp = self.get_current_pose()
    ce = self.get_end_effector_link()

    consts = moveit_msgs.msg.Constraints()
    oc = moveit_msgs.msg.OrientationConstraint()
    oc.header.frame_id = cp.header.frame_id
    oc.link_name = ce
    oc.orientation = cp.pose.orientation
    oc.absolute_x_axis_tolerance = 0.01
    oc.absolute_y_axis_tolerance = 0.01
    oc.absolute_z_axis_tolerance = 0.01
    oc.weight = 50
    consts.orientation_constraints = [oc]
    
    self.set_path_constraints(consts)
    s_idx = self.go_to_pose_goal(trans, rot, idx)
    self.clear_path_constraints()

    return s_idx

  def _grasp_to_pregrasp(self, g_trans, g_rot, g_offset, y_offset = 0):
    g_mat = tf_to_mat(g_trans, g_rot)
    gp_mat = tf_to_mat([0, y_offset, -g_offset], [0, 0, 0, 1])
    p_mat = tf.transformations.concatenate_matrices(g_mat, gp_mat)
    (p_trans, p_rot) = mat_to_tf(p_mat)

    return p_trans, p_rot

  def _convert(self, g_trans, g_target):
    g_mat = tf_to_mat(g_trans[:3], g_trans[3:])
    gp_mat = tf_to_mat(g_target[:3], g_target[3:])
    p_mat = tf.transformations.concatenate_matrices(g_mat, gp_mat)
    (p_trans, p_rot) = mat_to_tf(p_mat)

    return p_trans, p_rot

  def move_to_grab_part(self, g_trans, g_rot, g_offset, c, _check = False):
    '''
    [output]
    if plan0 and plan1 and plan2 and plan3 succeeded,
      return True
    else,
      return False
    '''
    (pg_trans, pg_rot) = self._grasp_to_pregrasp(g_trans, g_rot, g_offset)

    #print "pre grasp pose| " + list_str(pg_trans, ['x','y','z']) \
    #        + list_str(pg_rot, ['x','y','z','w'])
    #print "grasp pose| " + list_str(g_trans, ['x','y','z']) \
    #        + list_str(g_rot, ['x','y','z','w'])

    (result0, _) = self._get_best_ik_plan(g_trans, g_rot, c)
    print "******plan0 = {}\n".format(result0)
    if result0 < 0: return False


    if _check is not True: 

      result1 = self.go_to_pose_goal(pg_trans, pg_rot, result0)
  
     # print "******plan1 = {}\n".format(result1)
      if result1 < 0: return False
    
    # result2 = self.go_linear_to_pose_goal(g_trans, g_rot, result0)
    # print "******plan2 = {}\n".format(result2)
    # if result2 < 0: return False
    
    # result3 = self.go_linear_to_pose_goal(pg_trans, pg_rot, result0)
    # print "******plan3 = {}\n".format(result3)
    # if result3 < 0: return False
    
    return True

  def move_to_hold_part(self, g_trans, g_rot, g_offset, c):
    '''
    [output]
    if plan0 and plan1 and plan2 and plan3 succeeded,
      return True
    else,
      return False
    '''
    (pg_trans, pg_rot) = self._grasp_to_pregrasp(g_trans, g_rot, g_offset)

    # print "pre grasp pose| " + list_str(pg_trans, ['x','y','z']) \
    #         + list_str(pg_rot, ['x','y','z','w'])
    # print "grasp pose| " + list_str(g_trans, ['x','y','z']) \
    #         + list_str(g_rot, ['x','y','z','w'])

    (result0, _) = self._get_best_ik_plan(g_trans, g_rot, c)
    # print "******plan0 = {}\n".format(result0)
    if result0 < 0: return False

    result1 = self.go_to_pose_goal(pg_trans, pg_rot, result0)
    # print "******plan1 = {}\n".format(result1)
    if result1 < 0: 
      (pg_trans, pg_rot) = self._grasp_to_pregrasp(g_trans, g_rot, 0, y_offset = 0.3)
      result1y = self.go_to_pose_goal(pg_trans, pg_rot, result0)
      if result1y < 0:
        (pg_trans, pg_rot) = self._grasp_to_pregrasp(g_trans, g_rot, 0, y_offset = -0.3)
        result1y_ = self.go_to_pose_goal(pg_trans, pg_rot, result0)
        if result1y_ < 0:
          return False
    
    result2 = self.go_linear_to_pose_goal(g_trans, g_rot, result0)
    # print "******plan2 = {}\n".format(result2)
    if result2 < 0: return False
    
    # result3 = self.go_linear_to_pose_goal(pg_trans, pg_rot, result0)
    # print "******plan3 = {}\n".format(result3)
    # if result3 < 0: return False
    
    return True

def main():
  rospy.init_node('move_group_wrapper_test', anonymous=True)
  mg_rob1 = MoveGroupCommanderWrapper('rob1_arm', 'rob1_real_ee_link')
  mg_rob2 = MoveGroupCommanderWrapper('rob2_arm')
  # assembly_scene = Parts('part2') ###############
  listener = tf.TransformListener()

  print "ENTER"
  raw_input()

  mg_rob1.go_to_initial_pose()
  print "ENTER"
  raw_input()

  target_q = mg_rob1.get_named_target_values('rob1_pin1_camera_check')
  mg_rob1.go(target_q)
  print "ENTER"
  raw_input()
  
  mg_rob2.go_to_initial_pose()
  print "ENTER"
  raw_input()

  assembly_scene.add_mesh(PART2_PATH)
  print "ENTER"
  raw_input()

  assembly_scene.send_full_grasp_pose('part2')
  print "ENTER"
  raw_input()

  try:
    mg_rob1.set_end_effector_link('rob1_tcp_gripper_closed')
        
    print "ENTER"
    raw_input()

    grasp_offset = 0.2

    (grasp_trans1, grasp_rot1) \
     = listener.lookupTransform('rob1_real_base_link', 'part2_grasp', rospy.Time(0))
    grab_result = mg_rob1.move_to_grab_part(grasp_trans1, grasp_rot1, grasp_offset)
    print "move_to_grab_part :", grab_result
        
    print "ENTER"
    raw_input()

    mg_rob1.go_to_initial_pose()
    print "ENTER"
    raw_input()


    (grasp_trans2, grasp_rot2) \
      = listener.lookupTransform('rob2_real_base_link', 'part2_grasp', rospy.Time(0))

    grab_result = mg_rob2.move_to_grab_part(grasp_trans2, grasp_rot2, grasp_offset)
    print "move_to_grab_part :", grab_result

  except Exception as e:
    print e
  
  print "ENTER"
  raw_input()

if __name__ == '__main__':
  main()
