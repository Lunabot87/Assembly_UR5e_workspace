#!/usr/bin/env python
# execfile('move_to_test.py')
import rospy
import moveit_commander
import moveit_msgs.msg
rospy.init_node('test', anonymous=True)
mg1 = moveit_commander.MoveGroupCommander('rob1_arm')

cp = mg1.get_current_pose()
ce = mg1.get_end_effector_link()

const = moveit_msgs.msg.Constraints()
oconst = moveit_msgs.msg.OrientationConstraint()
oconst.header.frame_id = cp.header.frame_id
oconst.link_name = ce
oconst.orientation = cp.pose.orientation
oconst.absolute_x_axis_tolerance = 0.01
oconst.absolute_y_axis_tolerance = 0.01
oconst.absolute_z_axis_tolerance = 0.01
oconst.weight = 10
const.orientation_constraints = [oconst]

cp.pose.position.z -= 0.3

# mg1.set_path_constraints(const)
# mg1.set_pose_target(cp)
# mg1.go()