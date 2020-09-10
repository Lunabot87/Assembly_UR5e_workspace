#!/usr/bin/env python
import rospy

import geometry_msgs.msg
import moveit_commander

from Parts_hj import*

part_dir = "/home/hj/ur_ws/src/Assembly_UR5e_workspace/assembly_robot_description/stl/stefan_chair_part/"
part1_path = part_dir + "chair_part1.SLDPRT.STL"
part2_path = part_dir + "chair_part2.SLDPRT.STL"
part3_path = part_dir + "chair_part3.SLDPRT.STL"
part4_path = part_dir + "chair_part4.SLDPRT.STL"
part5_path = part_dir + "chair_part5.SLDPRT.STL"
part6_path = part_dir + "chair_part6.SLDPRT.STL"

rospy.init_node('test', anonymous=True)

ch = Parts('part2')
print "ENTER"
raw_input()

ch.add_mesh(part2_path)
print "ENTER"
raw_input()

# scene = moveit_commander.PlanningSceneInterface()
# mesh_pose = geometry_msgs.msg.PoseStamped()
# mesh_pose.header.frame_id = "table"
# mesh_pose.pose.orientation.x = 0
# mesh_pose.pose.orientation.y = 0
# mesh_pose.pose.orientation.z = 0
# mesh_pose.pose.orientation.w = 1

# print "ENTER"
# raw_input()

# print "is_added?", scene.add_mesh('part2', mesh_pose, part2_path, size=(1, 1, 1))

print "ENTER"
raw_input()