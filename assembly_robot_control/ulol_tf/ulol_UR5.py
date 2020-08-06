
import sys
import copy
import random
import roslib
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, sqrt, acos, sin, cos
from numpy.linalg import inv as Inv
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from nav_msgs.msg import Odometry
from tf.transformations import*
from tf import*

from part_info import*          # part_file_address, part_name

def make_orientation_list(orientation):
    o_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    return o_list
def make_position_list(position):
    p_list = [position.x,position.y,position.z]
    return p_list
def pose_from_transrot(trans = [0,0,0],rot = [0,0,0]):
    pose = geometry_msgs.msg.PoseStamped().pose
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]

    o_list = quaternion_from_euler(rot[0],rot[1],rot[2])
    pose.orientation.x = o_list[0]
    pose.orientation.y = o_list[1]
    pose.orientation.z = o_list[2]
    pose.orientation.w = o_list[3]

    return pose

class UR5(object):
    def __init__(self,G_name = 'rob1_arm'):
        super(UR5, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ulol_UR5', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_names = self.robot.get_group_names()

        self.group_name = G_name
        
        self.real_base_name = G_name.split('_')[0]+'_real_base_link'

        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
     
    def send_TF(self,pose,tf_name):
        position_list = make_position_list(pose.position)
        orientation_list = make_orientation_list(pose.orientation)
        self.br.sendTransform(position_list, orientation_list, rospy.Time.now(), tf_name, 'world')

    def go_to_initial_pose(self):
        joint_goal = [-pi/2,-pi/2,-pi/2,-pi/4,pi/2,pi]
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

    def move_cartesian_path(self, pose):
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

    def go_to_grab_ready_pose(self, pose, z_offset):
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

    # def rotate_ee_by_base(self,rot,br):
    #     base_rot = self.move_group.get_current_rpy(self.real_base_name)
    #     base_rot_matrix = euler_matrix(base_rot[0],base_rot[1],base_rot[2])[:3,:3]
    #     base_quaternion = quaternion_from_euler(base_rot[0],base_rot[1],base_rot[2])
    #     br.sendTransform([5,0,0], base_quaternion, rospy.Time.now(), 'base', 'world')
    #     if not rot == [0,0,0]:
    #         rotating_matrix = euler_matrix(rot[0],rot[1],rot[2])[:3,:3]
    #         new_rotation_matrix = base_rot_matrix.dot(rotating_matrix)

    #         ee_rot = self.move_group.get_current_rpy(self.eef_link)
    #         ee_rot_matrix = euler_matrix(ee_rot[0],ee_rot[1],ee_rot[2])[:3,:3]
    #         ee_quaternion = quaternion_from_euler(ee_rot[0],ee_rot[1],ee_rot[2])
    #         br.sendTransform([5,0,2], ee_quaternion, rospy.Time.now(), 'ee', 'world')

    #         Rot_ee_from_base = ee_rot_matrix.dot(Inv(base_rot_matrix))
    #         efbr = euler_from_matrix(Rot_ee_from_base)
    #         efbrq = quaternion_from_euler(efbr[0],efbr[1],efbr[2])
    #         br.sendTransform([5,0,4], efbrq, rospy.Time.now(), 'efbr', 'world')

    #         new_ee_matrix = (new_rotation_matrix).dot(Rot_ee_from_base)
    #         new_rot = euler_from_matrix(new_ee_matrix)

    #         new_pose = pose_from_transrot(rot = new_rot)
    #     else:
    #         new_pose = self.move_group.get_current_pose().pose

    #     return new_pose, base_rot_matrix

    # def move_ee_by_base(self,trans,rot = [0,0,0],br):#trans = [x,y,z], rot = [r,p,y]

    #     base_rot = self.move_group.get_current_rpy(self.real_base_name)
    #     base_rot_matrix = euler_matrix(base_rot[0],base_rot[1],base_rot[2])[:3,:3]
    #     new_trans = base_rot_matrix.dot(trans)

    #     (pose, base_matrix) = self.rotate_ee_by_base(rot,br)
    #     pose.position = self.move_group.get_current_pose().pose.position
    #     for i in range(3):
    #         pose.position.x += trans[0]*base_matrix[0,i]
    #         pose.position.y += trans[1]*base_matrix[1,i]
    #         pose.position.z += trans[2]*base_matrix[2,i]

    #     return pose


    def get_toolpose_from_base_1(self,input_axis,tool_rpy,base_rpy):
    # print relative vector of axis of tool by rob_real_base
        print "INPUT = ",input_axis
        axis = ['x','y','z']
        base = euler_matrix(base_rpy[0],base_rpy[1],base_rpy[2])
        tool = euler_matrix(tool_rpy[0],tool_rpy[1],tool_rpy[2])

        TF_base_from_tool = Inv(base).dot(tool)

        for a in input_axis:
            print " base_"+a+": [",
            for i in range(3):
                print '{:.3f}'.format(TF_base_from_tool[axis.index(a),i]),
            print "] in tool"


def example_for_test():
    try:
        R1 = UR5()
        R2 = UR5('rob2_arm')
        br = TransformBroadcaster()
        R1.go_to_initial_pose()
        R2.go_to_initial_pose()
        print "==========R1==========="
        print "Enter to Move",
        raw_input()
        joint_goal = [-pi/2,-pi/2,-pi/2,-pi/2,pi/2,pi]
        R1.go_to_joint_goal(joint_goal)

        tool_rpy = R1.move_group.get_current_rpy(R1.eef_link)
        base_rpy = R1.move_group.get_current_rpy(R1.real_base_name)
        R1.get_toolpose_from_base_1(['x','z'],tool_rpy,base_rpy)
        
        print "==========R2==========="
        print "Enter to Move",
        raw_input()
        joint_goal = [-pi/2,-pi/2-pi/4,-pi/2,
        pi/4,pi/2,pi]
        R2.go_to_joint_goal(joint_goal)

        tool_rpy = R2.move_group.get_current_rpy(R2.eef_link)
        base_rpy = R2.move_group.get_current_rpy(R2.real_base_name)
        R2.get_toolpose_from_base_1(['x','y'],tool_rpy,base_rpy)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    example_for_test()