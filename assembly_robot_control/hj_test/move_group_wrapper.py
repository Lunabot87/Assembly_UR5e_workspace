#!/usr/bin/env python
import tf
import rospy
import time
import math as m
import copy 
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

from ur5e_inv_kin_wrapper import UR5eInvKinForTF
from Parts_hj import*
from const import *

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
      eef_link = 'tcp_gripper_closed'
      _eef_link = name[:5] + eef_link
    else:
      eef_link = _eef_link[5:]    
    MoveGroupCommander.set_end_effector_link(self, _eef_link)
    
    # setup for ur5e invkin
    eef_offset = TCP_OFFSET[eef_link]
    self.ur5e = UR5eInvKinForTF(eef_offset)
    print"eef_link: {}, eef_offset: {}".format(eef_link, eef_offset)

  def get_name(self):
    '''
    how to handle method overloading
    '''
    print "My name is {}".format(self.__class__.__name__)
    # return super(MoveGroupCommanderWrapper, self).get_name()
    return MoveGroupCommander.get_name(self)

  def _list_to_js(self, joint_list):
    js = JointState()
    js.name = self.get_active_joints()
    js.position = joint_list
    return js

  def set_end_effector_link(self, _eef_link):
    MoveGroupCommander.set_end_effector_link(self, _eef_link)
    
    eef_link = _eef_link[5:]
    eef_offset = TCP_OFFSET[eef_link]
    self.ur5e.set_eef_offset(eef_offset)

  def go_to_initial_pose(self):
    joint_goal = [-m.pi/2, -m.pi/2,-m.pi/2,-m.pi/2, m.pi/2, m.pi]
    self.go(joint_goal, wait=True)
    self.stop()

  def go_to_pose_goal(self, trans, rot):
    cur_joint = self.get_current_joint_values()
    inv_sol = self.ur5e.inv_kin_full_sorted(trans, rot, cur_joint)
    self.ur5e.print_inv_sol(inv_sol)
    
    print "="*100
    print "current q: ", cur_joint
    for i in range(8):
      if inv_sol[i]['valid']:
        selected_q = (inv_sol[i]['inv_sol'])
        print "selected q: ", selected_q
        self.ur5e.publish_state(selected_q, True)
        (success, traj, b, err) = self.plan(self._list_to_js(selected_q))
        if success:
          print "plan success: {}".format(b)
          user_choice = raw_input("--> press [y/n]")
          if user_choice == 'y':
            self.execute(traj, wait=True)
            self.stop()
            return True
          else:
            continue
        else:
          print "plan error:", b, err
    
    return False

  def go_linear_to_pose_goal(self, trans, rot):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    
    waypoint = []
    waypoint.append(pose)
    (plan, fraction) = self.compute_cartesian_path(waypoint,0.01,0.0)
    if plan:
      print "ENTER"
      raw_input()
      self.execute(plan, wait = True)
      return True
    else:
      return False

  def move_to_grasp_part(self):
    '''
    offset : z dir wrt grasp pose
    '''
    pass
    

def list_str(some_list, some_key):
  msg = ""  
  try:
    for i in range(len(some_list)):
      msg += "{}: {:.3f} ".format(some_key[i], some_list[i])
  except:
    print "failed to make list look pretty!"

  return msg

def main():
  rospy.init_node('move_group_wrapper_test', anonymous=True)
  mg_rob1 = MoveGroupCommanderWrapper('rob1_arm', 'rob1_real_ee_link')
  assembly_scene = Parts('part2')
  listener = tf.TransformListener()

  print "ENTER"
  raw_input()

  mg_rob1.go_to_initial_pose()
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
    mg_rob1.set_pose_reference_frame('table')
    print "ENTER"
    raw_input()
    print "planning frame:", mg_rob1.get_pose_reference_frame()

    # mg_rob1.move_to_grasp_part('part2')
    (pre_grasp_pose_trans, pre_grasp_pose_rot) \
      = listener.lookupTransform('rob1_real_base_link', 'part2_pre_grasp', rospy.Time(0))
    (grasp_pose_trans, grasp_pose_rot) \
      = listener.lookupTransform('table', 'part2_grasp', rospy.Time(0))
    (post_grasp_pose_trans, post_grasp_pose_rot) \
      = listener.lookupTransform('table', 'part2_pre_grasp', rospy.Time(0))
    print "pre grasp pose| " + list_str(pre_grasp_pose_trans, ['x','y','z']) \
            + list_str(pre_grasp_pose_rot, ['x','y','z','w'])
    print "grasp pose| " + list_str(grasp_pose_trans, ['x','y','z']) \
            + list_str(grasp_pose_rot, ['x','y','z','w'])
    print "post grasp pose| " + list_str(post_grasp_pose_trans, ['x','y','z']) \
            + list_str(post_grasp_pose_rot, ['x','y','z','w'])

    result = mg_rob1.go_to_pose_goal(pre_grasp_pose_trans, pre_grasp_pose_rot)
    print "******plan1=", result
    result = mg_rob1.go_linear_to_pose_goal(grasp_pose_trans, pre_grasp_pose_rot)
    print "******plan2=", result
    result = mg_rob1.go_linear_to_pose_goal(post_grasp_pose_trans, post_grasp_pose_rot)
    print "******plan3=", result
  except Exception as e:
    print e
  
  print "ENTER"
  raw_input()

if __name__ == '__main__':
  main()
