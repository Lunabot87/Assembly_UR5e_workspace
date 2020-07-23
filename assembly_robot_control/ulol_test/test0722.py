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
scene = moveit_commander.PlanningSceneInterface()

part1 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part1.SLDPRT.STL"
part2 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part2.SLDPRT.STL"
part3 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part3.SLDPRT.STL"
part4 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part4.SLDPRT.STL"
part5 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part5.SLDPRT.STL"
part6 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part6.SLDPRT.STL"

def rearrange_solution(ik_solution):
    print ik_solution
    new_sol = IKW.ur5.np.zeros((8,6))
    print new_sol
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
    
  def attach_part(self, mesh_name, file, grasping_group = 'rob1_hand'):
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_mesh(self.eef_link, mesh_name, filename = file, size = (1,1,1),touch_links=touch_links)

  def detach_part(self, mesh_name): 
    self.scene.remove_attached_object(self.eef_link, mesh_name)

# class Parts(object):
  # def __init__(self, part_name):
  #   super(Parts, self).__init__()
  #   self.scene = scene
  #   self.part_name = part_name
  #   self.xyz_length = [0.34,0.05,0.019]

  # def get_mesh_pose(self, mesh_name):
  #   mesh_pose = geometry_msgs.msg.PoseStamped()
  #   mesh_pose.header = self.scene.get_objects([mesh_name])[mesh_name].header
  #   mesh_pose.pose = self.scene.get_objects([mesh_name])[mesh_name].mesh_poses[0]
  #   return mesh_pose

  # def add_mesh(self, file_name):
  #   mesh_name = self.part_name
  #   mesh_pose = geometry_msgs.msg.PoseStamped()

  #   mesh_pose.header.frame_id = "world"

  #   temp_r = random.uniform(-3.14,3.14)
  #   orientation_list = quaternion_from_euler(0,0,temp_r)
  #   mesh_pose.pose.position.x = random.uniform(-0.4,0.3)
  #   mesh_pose.pose.position.y = random.uniform(0, 0.4)
  #   mesh_pose.pose.position.z = 0.81
  #   mesh_pose.pose.orientation.x = orientation_list[0]
  #   mesh_pose.pose.orientation.y = orientation_list[1]
  #   mesh_pose.pose.orientation.z = orientation_list[2]
  #   mesh_pose.pose.orientation.w = orientation_list[3]
  #   print mesh_pose.pose

  #   self.scene.add_mesh(mesh_name, mesh_pose, file_name, size=(1, 1, 1))
  #   print "[INFO] MESH NAME = '",mesh_name,"' ADDED"

  # def rem_mesh(self, mesh_name):
  #   self.scene.remove_world_object(mesh_name)
  #   print "[INFO] MESH NAME = '",mesh_name,"' REMOVED"

  # def __del__(self):
  #   self.scene.remove_world_object()

def main():
  try:
    
    print ""
    print "----------------------------------------------------------"
    print ""
    ROB1 = UR5()
    UR = IKW.ur5.ur5()
    print "[[[[[[[initialize DONE]]]]]]]]]]"
    while True:
        print "============ Press `Enter`  to get_current_pose"
        raw_input()
        pose = ROB1.move_group.get_current_pose().pose
        pose.position.y -= 0.745
        pose.position.z -= 0.8109
        target_T = make_transform_matrix(pose)
        current_joint = ROB1.move_group.get_current_joint_values()
        current_T = UR.fwd_kin(current_joint)
        solution = UR.inv_kin(target_T)
        solution = rearrange_solution(solution)
        print "current joint"
        print current_joint
        print "=============="
        print "FOWARD KINENATICS"
        print current_T
        print ""
        print "target_T"
        print target_T
        print ""
        print "solutions"
        print solution
        print "============ Press `Enter` TO MOVE"
        raw_input()
        ROB1.go_to_joint_goal(current_joint)
        for i in range(8):
            print "print Enter to go solution #",i
            raw_input()
            ROB1.go_to_joint_goal(solution[i])
        print "============ complete!" 
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return



if __name__ == '__main__':
    main()