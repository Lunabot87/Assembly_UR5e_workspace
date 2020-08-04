import tf
import sys
import copy
import random
import roslib
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from nav_msgs.msg import Odometry
from tf.transformations import*

from part_info import*          # part_file_address, part_name

def make_orientation_list(orientation):
  o_list = [orientation.x, orientation.y, orientation.z, orientation.w]
  return o_list
 
class UR5(object):
    def __init__(self,G_name = 'rob1_arm'):
        super(UR5, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_names = self.robot.get_group_names()

        self.group_name = G_name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
     
    def go_to_initial_pose(self):
        joint_goal = [-pi/2,-pi/2,-pi/2,-pi/2,pi/2,pi]
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def go_to_joint_goal(self,joint_goal):
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def go_to_pose_goal(self,pose_goal):
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def move_cartesian_path(self, pose):
        waypoint = []
        waypoint.append(copy.deepcopy(pose))
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoint,0.01,0.0)
        self.move_group.execute(plan,wait = True)

    def add_cartesian_pose(self, trans):
        pose = self.move_group.get_current_pose().pose
        pose.position.x += trans[0]
        pose.position.y += trans[1]
        pose.position.z += trans[2]
        waypoint = []
        waypoint.append(copy.deepcopy(pose))
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoint,0.01,0.0)
        self.move_group.execute(plan,wait = True)


    def get_target_orientation(self,orientation): # have to be modified in rotation
        
        orientation_list = make_orientation_list(orientation)
        Rotation = quaternion_matrix(orientation_list)
        rol_add = 3.14 #rx
        pit_add = 0 #ry
        yaw_add = 0 #rz

        Rotation_add = euler_matrix(rol_add,pit_add,yaw_add)
        Rotation_new = Rotation.dot(Rotation_add)

        (rol,pit,yaw) = euler_from_matrix(Rotation_new)
        o_list = quaternion_from_euler(rol,pit,yaw)
        new_orientation = geometry_msgs.msg.PoseStamped().pose.orientation
        new_orientation.x = o_list[0]
        new_orientation.y = o_list[1]
        new_orientation.z = o_list[2]
        new_orientation.w = o_list[3]

        return new_orientation

    def get_random_pose(self, x = random.uniform(-0.35,0.35), y = random.uniform(0,0.4) - 0.745
                                , z = random.uniform(0.2,0.3), roll = random.uniform(-3.14,3.14)
                                , pitch = random.uniform(-3.14,3.14), yaw = random.uniform(-3.14,3.14)):
        
        orientation_list = quaternion_from_euler(roll,pitch,yaw)

        pose = self.move_group.get_current_pose().pose
        pose = geometry_msgs.msg.PoseStamped().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = orientation_list[0]
        pose.orientation.y = orientation_list[1]
        pose.orientation.z = orientation_list[2]
        pose.orientation.w = orientation_list[3]

        return pose


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

    def grab_part(self, pose, z_offset):
        target_pose = copy.deepcopy(pose)
        target_pose.orientation = self.get_target_orientation(target_pose.orientation)

        o_list = make_orientation_list(target_pose.orientation)
        Rotation_Matrix = quaternion_matrix(o_list)
        z_axis = [Rotation_Matrix[0][2],Rotation_Matrix[1][2],Rotation_Matrix[2][2]] # elements of 3rd column
        target_pose.position.x -= z_offset * z_axis[0]
        target_pose.position.y -= z_offset * z_axis[1]
        target_pose.position.z -= z_offset * z_axis[2]
          
        self.move_group.set_pose_target(target_pose)
        self.move_group.go(wait=True)

        print "PRESS ENTER TO MOVE_DOWN"
        raw_input()
        target_pose.position.x += z_offset * z_axis[0]
        target_pose.position.y += z_offset * z_axis[1]
        target_pose.position.z += z_offset * z_axis[2]
        self.move_cartesian_path(target_pose)
        # print "[INFO] Ready to attach"    

        # print "PRESS ENTER TO ATTACH"
        # raw_input()        
        # self.attach_part(attach_list)
        # print "[INFO] Object attached"

        # print "PRESS ENTER TO MOVE-UP"
        # raw_input()
        # target_pose.position.z += 0.3
        # target_pose.position.x += 0.1
        # self.move_cartesian_path(target_pose)

        # print "PRESS ENTER TO MOVE_DOWN"
        # raw_input()
        # target_pose.position.z -= z_offset
        # self.move_cartesian_path(target_pose)
        # print "[INFO] Ready to detach"


        # print "PRESS ENTER TO DETACH"
        # raw_input()
        # self.detach_part(attach_list)

        
    def attach_part(self, attach_list):
        if self.group_name == 'rob1_arm':
            grasping_group = 'rob1_hand'
        elif self.group_name == 'rob2_arm':
            grasping_group = 'rob2_hand'

        touch_links = self.robot.get_link_names(group=grasping_group)
        
        for attaching_part_name in attach_list['part']:
            mesh_file = part_file[part_name.index(attaching_part_name)]
            self.scene.attach_mesh(self.eef_link, attaching_part_name, filename = mesh_file, size = (1,1,1), touch_links = touch_links)
        
        for attaching_pin_name in attach_list['pin']:

            temp_pin_name = attaching_part_name.split('-')[0]

            mesh_file = part_file[part_name.index(temp_pin_name)]
            self.scene.attach_mesh(self.eef_link, attaching_pin_name, filename = mesh_file, size = (1,1,1), touch_links = touch_links)

    def detach_part(self, attach_list):
        for attaced_part_name in attach_list['part']:
            self.scene.remove_attached_object(self.eef_link, attaced_part_name)
        for attaced_pin_name in attach_list['pin']:
            self.scene.remove_attached_object(self.eef_link, attaced_pin_name)

    # def attached_part(self,name=[]):
    #     A = self.scene.get_attached_objects(name)
    #     print A