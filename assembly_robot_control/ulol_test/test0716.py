#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from math import sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
from time import sleep

part1 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part1.SLDPRT.STL"
part2 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part2.SLDPRT.STL"
part3 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part3.SLDPRT.STL"
part4 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part4.SLDPRT.STL"
part5 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part5.SLDPRT.STL"
part6 = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_chair_stl/chair_part6.SLDPRT.STL"

def get_eef_orientation(orientation):
	w = orientation.w
	x = orientation.x
	y = orientation.y
	z = orientation.z
	(r,p,y) = euler_from_quaternion([x,y,z,w])

	orientation_list = quaternion_from_euler(r+pi,p,y)

	orientation.x = orientation_list[0]
	orientation.y = orientation_list[1]
	orientation.z = orientation_list[2]
	orientation.w = orientation_list[3]

	return orientation


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
    rospy.init_node('test0716', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    group_names = robot.get_group_names()
    self.group_name = G_name
    move_group = moveit_commander.MoveGroupCommander(self.group_name)
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()


    self.__num_of_mesh = 0
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

 
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
    pose_goal.position.x = 0.0
    pose_goal.position.y = 0.0
    pose_goal.position.z = 1.0

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()

    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):

    move_group = self.move_group
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))


    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    return plan, fraction

  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan, wait=True)





  def grab_part(self, target_name, target_file, target_pose, offset):
  	#NotImplementedError

  	waypoint = []
	waypoint.append(copy.deepcopy(target_pose))

	Rotation_Matrix = quaternion_matrix(target_pose.pose.orientation)
	z_axis = [Rotation_Matrix[0][2],Rotation_Matrix[1][2],Rotation_Matrix[2][2]] # elements of 3rd column

	target_pose.position.x -= offset * z_axis[0]
	target_pose.position.y -= offset * z_axis[1]
	target_pose.position.z -= offset * z_axis[2]
	temp_pose = target_pose

	self.move_group.set_pose_target(target_pose)
	self.move_group.go(wait=True)
	print "target_pose-offset CLEAR"
	sleep(1.0)

	waypoint.append(copy.deepcopy(pose_goal))
  	(plan, fraction) = self.move_group.compute_cartesian_path(waypoint,0.01,0.0)
  	self.move_group.execute(plan,wait = True)
  	print "target_pose CLEAR"
  	sleep(1.0)

  	grasping_group = 'rob1_hand'
  	touch_links = self.robot.get_link_names(group=grasping_group)
	self.scene.attach_mesh(self.eef_link, target_name, filename = target_file, size = (1,1,1),touch_links=touch_links)
	print "object attached"

	sleep(1.0)


	waypoint = []
	waypoint.append(copy.deepcopy(temp_pose))
	waypoint.append(copy.deepcopy(pose_goal))
  	(plan, fraction) = self.move_group.compute_cartesian_path(waypoint,0.01,0.0)
  	self.move_group.execute(plan,wait = True)


  def move_to_part(self,part_pose):
 #  	dist = 0.2
 #  	pose_goal = part_pose.pose
  	
 #  	orientation = part_pose.pose.orientation
 #  	e1 = orientation.x
 #  	e2 = orientation.y
 #  	e3 = orientation.z
 #  	e4 = orientation.w
 #  	z_axis = [2*(e1*e3+e2*e4),2*(e2*e3-e1*e4),1-2*e1*e1-2*e2*e2]

 #  	temp = pose_goal.position
 #  	pose_goal.position.x += z_axis[0]*dist
 #  	pose_goal.position.y += z_axis[1]*dist
 #  	pose_goal.position.z += z_axis[2]*dist

 #  	pose_goal.orientation = get_eef_orientation(orientation)
	# self.move_group.set_pose_target(pose_goal)
 #  	self.move_group.go(wait=True)
	
	# waypoint=[]
	# pose_goal.position = temp
 #  	waypoint.append(copy.deepcopy(pose_goal))
 #  	(plan, fraction) = self.move_group.compute_cartesian_path(waypoint,0.01,0.0)
 #  	self.move_group.execute(plan,wait = True)


#  def attach_mesh(self, mesh_name, file, grasping_group = 'rob1_hand'):
#	touch_links = self.robot.get_link_names(group=grasping_group)
#	self.scene.attach_mesh(self.eef_link, mesh_name, filename = file, size = (1,1,1),touch_links=touch_links)

#  def detach_mesh(self, mesh_name):	
#    self.scene.remove_attached_object(self.eef_link, mesh_name)

class Parts(object):
	def __init__(self):
    super(Parts, self).__init__()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.__num_of_mesh = 0

  def get_num_of_meshes(self):
  	return self.__num_of_mesh

  def get_mesh_pose(self, mesh_name):
  	mesh_pose = geometry_msgs.msg.PoseStamped()
  	mesh_pose.header = self.scene.get_objects([mesh_name])[mesh_name].header
  	mesh_pose.pose = self.scene.get_objects([mesh_name])[mesh_name].mesh_poses[0]
  	return mesh_pose

  def add_mesh(self, file_name):
  	self.__num_of_mesh += 1
	mesh_name = 'part' + str(self.__num_of_mesh)
	
	mesh_pose = geometry_msgs.msg.PoseStamped()
	mesh_pose.header.frame_id = "world"
	mesh_pose.pose.orientation.x = 0.0
	mesh_pose.pose.orientation.y = 0.0
	mesh_pose.pose.orientation.z = 0.0
	mesh_pose.pose.orientation.w = 1.0
	mesh_pose.pose.position.z = 0.83 # slightly above the end effector
	self.scene.add_mesh(mesh_name, mesh_pose, file_name, size=(1, 1, 1))
	print "[INFO] MESH NAME = '",mesh_name,"' ADDED"

  def rem_mesh(self, mesh_name):
  	self.scene.remove_world_object(mesh_name)
  	print "[INFO] MESH NAME = '",mesh_name,"' REMOVED"

  def get_mesh_z(self, mesh_name):
  	orientation = get_mesh_pose(mesh_name).pose.orientation
  	e1 = orientation.x
  	e2 = orientation.y
  	e3 = orientation.z
  	e4 = orientation.w
  	z_axis = [2*(e1*e3+e2*e4),2*(e2*e3-e1*e4),1-2*e1*e1-2*e2*e2]

  def __del__(self):
  	self.scene.remove_world_object()

def main():
  try:
	print ""
	print "----------------------------------------------------------"
	print ""
	print "============ Press `Enter` ...1"
	raw_input()
	ROB1 = UR5()
	ROB2 = UR5('rob2_arm')
	Ch = Parts()
	Ch.scene.remove_world_object()
	print "[[[[[[[initialize doen]]]]]]"
	print "============ Press `Enter` ...2"
	raw_input()
	ROB1.go_to_joint_state()
	ROB2.go_to_joint_state()
	Ch.add_mesh(part2)
	print "[[[[[[[mesh add done]]]]]]"


	print "============ Press `Enter` ...3"
	raw_input()
	ROB1.move_to_part('part1',part2)

	# print "============ Press `Enter` ...3"
	# raw_input()
	# pose_goal = Ch.get_mesh_pose('part1')
	# print "[[[[[[[get mesh_pose doen]]]]]]"

	# print "============ Press `Enter` ...4"
	# raw_input()
	# ROB1.move_to_part(pose_goal)

	# print "============ Press `Enter` ...5"
	# raw_input()
	# ROB1.attach_mesh('part1',part2)

	# print "============ Press `Enter` ..."
	# raw_input()
	# ROB1.go_to_pose_goal()

	# print "============ Press `Enter` ..."
	# raw_input()
	# ROB1.detach_mesh('part1')
	# ROB1.go_to_joint_state()

	print "============ complete!"
  except rospy.ROSInterruptException:
  	return
  except KeyboardInterrupt:
  	return
if __name__ == '__main__':
 main()