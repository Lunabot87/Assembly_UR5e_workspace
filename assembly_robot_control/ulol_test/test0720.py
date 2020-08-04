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

from math import pi
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

def make_oriention_list(orientation):
  o_list = [orientation.x, orientation.y, orientation.z, orientation.w]
  return o_list
 
def handle_mesh_pose(part_name, target_pose):
  br = tf.TransformBroadcaster()
  p_list = [target_pose.position.x, target_pose.position.y, target_pose.position.z]
  o_list = make_oriention_list(target_pose.orientation)
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
 
  def go_to_joint_state(self):
    move_group = self.move_group
    joint_goal = [-pi/2,-pi/2,-pi/2,-pi/2,pi/2,pi/2]
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):

    move_group = self.move_group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 1.0
    pose_goal.position.z = 1.0

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()

    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

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
        pose.position.z = 0.9 + target_size[0]/2 + offset
        (r,p) = (pi/2,pi)
    else:
        pose.position.z = 0.9 + target_size[1]/2 + offset
        (r,p) = (pi/2,pi/2)
    y = yaw

    orientation_list = quaternion_from_euler(r, p, y)
    pose.orientation.x = orientation_list[0]
    pose.orientation.y = orientation_list[1]
    pose.orientation.z = orientation_list[2]
    pose.orientation.w = orientation_list[3]

    return pose


  def grab_part(self, target_name, target_file, target_pose, target_size, offset):
    o_list = make_oriention_list(target_pose.orientation)
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
    target_pose = self.get_erected_pose(target_pose,target_size, offset, yaw = 0)
    self.move_group.set_pose_target(target_pose)
    self.move_group.go(wait=True)

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




class Parts(object):
  def __init__(self, part_name):
    super(Parts, self).__init__()
    self.scene = scene
    self.part_name = part_name
    self.xyz_length = [0.34,0.05,0.019]

  def get_mesh_pose(self, mesh_name):
  	mesh_pose = geometry_msgs.msg.PoseStamped()
  	mesh_pose.header = self.scene.get_objects([mesh_name])[mesh_name].header
  	mesh_pose.pose = self.scene.get_objects([mesh_name])[mesh_name].mesh_poses[0]
  	return mesh_pose

  def add_mesh(self, file_name):
    mesh_name = self.part_name
    mesh_pose = geometry_msgs.msg.PoseStamped()

    mesh_pose.header.frame_id = "world"

    temp_r = random.uniform(-3.14,3.14)
    temp_olist = quaternion_from_euler(0,0,temp_r)
    mesh_pose.pose.position.x = random.uniform(-0.4,0.4)
    mesh_pose.pose.position.y = random.uniform(0, 0.4)
    mesh_pose.pose.position.z = 0.81
    mesh_pose.pose.orientation.x = temp_olist[0]
    mesh_pose.pose.orientation.y = temp_olist[1]
    mesh_pose.pose.orientation.z = temp_olist[2]
    mesh_pose.pose.orientation.w = temp_olist[3]

    # mesh_pose.pose.position.z = 0.8
    # mesh_pose.pose.orientation.w = 1.0

    self.scene.add_mesh(mesh_name, mesh_pose, file_name, size=(1, 1, 1))
    print "[INFO] MESH NAME = '",mesh_name,"' ADDED"

  def rem_mesh(self, mesh_name):
  	self.scene.remove_world_object(mesh_name)
  	print "[INFO] MESH NAME = '",mesh_name,"' REMOVED"

  def __del__(self):
  	self.scene.remove_world_object()

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print ""
    # print "============ Press `Enter` ...1"
    # raw_input()
    ROB1 = UR5()
    ROB2 = UR5('rob2_arm')
    Ch = Parts('chair_part1')
    scene.remove_world_object()
    print "[[[[[[[initialize DONE]]]]]]]]]]"
    print "============ Press `Enter` ...2"
    raw_input()
    ROB1.go_to_joint_state()
    ROB2.go_to_joint_state()
    Ch.add_mesh(part2)
    print "[[[[[[[mesh add done]]]]]]"
    
    print "============ Press `Enter` ...3"
    raw_input()
    mesh_pose = Ch.get_mesh_pose('chair_part1').pose
    position_list = [mesh_pose.position.x, mesh_pose.position.y, mesh_pose.position.z]
    orientation_list = make_oriention_list(mesh_pose.orientation)

    br = tf.TransformBroadcaster()
    br.sendTransform(position_list, orientation_list, rospy.Time.now(), 'part1', 'world')

    mesh_pose.orientation = ROB1.get_target_orientation(mesh_pose.orientation,Ch.xyz_length)

    ROB1.grab_part(target_name = 'chair_part1', target_file = part2, target_pose = mesh_pose, target_size = Ch.xyz_length, offset = 0.2)
    ROB1.detach_part('chair_part1')
    print "============ complete!"
    print "============ PRESS ENTER TO END"
    raw_input()
  
  except rospy.ROSInterruptException:
  	return
  except KeyboardInterrupt:
  	return
if __name__ == '__main__':
 main()
