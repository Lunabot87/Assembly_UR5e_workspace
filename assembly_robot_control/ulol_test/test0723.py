#!/usr/bin/env python
import tf
import sys
import copy
import random
import roslib
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import ur5_inv_kin_wrapper as IKW

from math import pi, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

def rearrange_solution(ik_solution):
    new_sol = IKW.np.zeros((8,6))
    for i in range(0,8):
        for j in range(0,6):
            new_sol[i][j] = ik_solution[j][i]
    return new_sol

def make_transform_matrix(pose):
  orientation_list = make_orientation_list(pose.orientation)
  T = quaternion_matrix(orientation_list)
  T[0][3] = pose.position.x 
  T[1][3] = pose.position.y
  T[2][3] = pose.position.z
  return T

def make_orientation_list(orientation):
  o_list = [orientation.x, orientation.y, orientation.z, orientation.w]
  return o_list
 
def handle_mesh_pose(part_name, target_pose):
  br = tf.TransformBroadcaster()
  p_list = [target_pose.position.x, target_pose.position.y, target_pose.position.z]
  o_list = make_orientation_list(target_pose.orientation)
  br.sendTransform(p_list,o_list,rospy.Time.now(),part_name,"world")

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class UR5(object):
  def __init__(self,G_name = 'rob1_arm'):
    super(UR5, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test0720', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()

    self.robot = moveit_commander.RobotCommander()
    self.scene = scene
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    group_names = self.robot.get_group_names()
    self.group_name = G_name
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
    self.planning_frame = self.move_group.get_planning_frame()
    self.eef_link = self.move_group.get_end_effector_link()
 
  def go_to_initial_pose(self):
    joint_goal = [-pi/2,-pi/2,-pi/2,-pi/2,pi/2,pi]
    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()

    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_goal(self,joint_goal):
    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self,pose):
    self.move_group.set_pose_target(pose)
    plan = self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def execute_plan(self, plan):
    self.move_group.execute(plan, wait=True)

  def move_cartesian_path(self, pose):
    waypoint = []
    waypoint.append(copy.deepcopy(pose))
    (plan, fraction) = self.move_group.compute_cartesian_path(waypoint,0.01,0.0)
    self.move_group.execute(plan,wait = True)

  def get_target_orientation(self,orientation,xyz_length):
    w = orientation.w
    x = orientation.x
    y = orientation.y
    z = orientation.z
    (r,p,y) = euler_from_quaternion([x,y,z,w])

    if xyz_length[0] >= xyz_length[1]:
        orientation_list = quaternion_from_euler(r+pi,p,y+pi/2)
    else:
        orientation_list = quaternion_from_euler(r+pi,p,y+pi)

    orientation.x = orientation_list[0]
    orientation.y = orientation_list[1]
    orientation.z = orientation_list[2]
    orientation.w = orientation_list[3]

    return orientation

  def get_erected_pose(self, current_pose, target_size, offset, yaw = 0):
    pose = current_pose
    pose.position.x = 0
    pose.position.y = 0.2

    if target_size[0] >= target_size[1]:
        pose.position.z += offset + target_size[0]/2
        (r,p) = (pi/2,pi)
    else:
        pose.position.z += offset + target_size[1]/2
        (r,p) = (pi/2,pi/2)
    y = yaw

    orientation_list = quaternion_from_euler(r, p, y)
    pose.orientation.x = orientation_list[0]
    pose.orientation.y = orientation_list[1]
    pose.orientation.z = orientation_list[2]
    pose.orientation.w = orientation_list[3]

    return pose

  def rotate_wrist(self,xyz_length):
    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    joint_goal[4] -= pi
    joint_goal[3] -= pi/2
    if xyz_length[0] >= xyz_length[1]:
      joint_goal[5] = pi
    else:
      joint_goal[5] = pi/2
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01) 

  def grab_part(self, target_name, target_file, target_pose, target_size, offset):

    o_list = make_orientation_list(target_pose.orientation)
    Rotation_Matrix = quaternion_matrix(o_list)
    z_axis = [Rotation_Matrix[0][2],Rotation_Matrix[1][2],Rotation_Matrix[2][2]] # elements of 3rd column
    target_pose.position.x -= offset * z_axis[0]
    target_pose.position.y -= offset * z_axis[1]
    target_pose.position.z -= offset * z_axis[2]
      
    self.move_group.set_pose_target(target_pose)
    self.move_group.go(wait=True)

    print "PRESS ENTER TO MOVE_DOWN"
    raw_input()
    target_pose.position.x += offset * z_axis[0]
    target_pose.position.y += offset * z_axis[1]
    target_pose.position.z += offset * z_axis[2]
    self.move_cartesian_path(target_pose)
    print "[INFO] Ready to attach"    

    print "PRESS ENTER TO ATTACH"
    raw_input()
    grasping_group = 'rob1_hand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.attach_part(target_name, target_file, grasping_group)
    print "[INFO] Object attached"

    print "PRESS ENTER TO MOVE-UP"
    raw_input()
    target_pose.position.z += 0.3
    self.move_cartesian_path(target_pose)

    print "PRESS ENTER TO ERECT THE PART"
    raw_input()
    target_pose = self.get_erected_pose(target_pose,target_size,offset,yaw = 0)

    self.rotate_wrist(target_size)

    print "PRESS ENTER TO MOVE_DOWN"
    raw_input()
    target_pose.position.z = 0.8 + 0.17 + 0.05
    target_pose.position.x = random.uniform(-0.3,0.3)
    target_pose.position.y = random.uniform(0, 0.3)
    print self.move_group.get_current_pose().pose.position
    self.move_cartesian_path(target_pose)

    print "PRESS ENTER TO MOVE_DOWN"
    raw_input()
    target_pose.position.z -= offset
    self.move_cartesian_path(target_pose)
    print "[INFO] Ready to detach"

def main():
  ROB1 = UR5()
  ur5_inv = IKW.ur5_inv_kin_wrapper()
  listener = tf.TransformListener()
  print "============ INITIALIZE DONE"

  while not rospy.is_shutdown():
    try:
      print "============ Press `Enter`  to get_current_joint_values"
      raw_input()
      cur_joint = ROB1.move_group.get_current_joint_values()
      (trans, rot) = listener.lookupTransform('/rob1_real_base_link', '/rob1_real_ee_link', rospy.Time(0))
      ur5_inv.solve_and_sort(trans, rot, cur_joint)
      A = []
      s = []
      j = 0
      ur5_inv._print_sol(ur5_inv.inv_sol)
      for i in range(8):
        rv = ur5_inv._get_state_validity(ur5_inv.inv_sol[:,i])
        if rv == True:
          j += 1
          s.append(i)
          A.append(ur5_inv.inv_sol[:,i])          

      ur5_inv.inv_sol = A
      raw_input()
      print "="*17+"VALID  SOLUTION"+"="*17
      for i in range(j):
          print 'sol #',s[i]
          print "PRESS ENTER"
          raw_input()
          ur5_inv.publish_state(i)
      print('='*50)
      print "complete!" 

    except rospy.ROSInterruptException:
      return
    except KeyboardInterrupt:
      return



if __name__ == '__main__':
    main()