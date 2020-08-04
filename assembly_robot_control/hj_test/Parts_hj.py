import tf
import rospy
import random
import math as m

import moveit_commander
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

from utils.conversions import *

br = tf.TransformBroadcaster()

class Parts():
  def __init__(self, part_name):
    self.scene = moveit_commander.PlanningSceneInterface()
    self.part_name = part_name
    self.xyz_length = [0.34,0.05,0.019]

  def get_mesh_pose(self, mesh_name):
    mesh_pose = geometry_msgs.msg.PoseStamped()
    mesh_pose.header = self.scene.get_objects([mesh_name])[mesh_name].header
    mesh_pose.pose = self.scene.get_objects([mesh_name])[mesh_name].mesh_poses[0]
    return mesh_pose

  def send_full_grasp_pose(self, part_name, offset=0.2):
    part_pose = self.get_mesh_pose(part_name)    
    pf = part_pose.header.frame_id
    pp = part_pose.pose.position
    po = part_pose.pose.orientation
    
    (r,p,y) = euler_from_quaternion([po.x,po.y,po.z,po.w])
    go_list = quaternion_from_euler(r+m.pi,p,y+m.pi/2)
    gp_list = [pp.x, pp.y, pp.z]

    g_mat = tf_to_mat(gp_list, go_list)
    gp_mat = tf_to_mat([0, 0, -offset], [0, 0, 0, 1])
    p_mat = tf.transformations.concatenate_matrices(g_mat, gp_mat)
    (pp_list, po_list) = mat_to_tf(p_mat)

    br.sendTransform(pp_list, po_list, rospy.Time.now(), part_name+'_pre_grasp', pf)
    br.sendTransform(gp_list, go_list, rospy.Time.now(), part_name+'_grasp', pf)

  def send_grasp_pose(self, part_name):
    part_pose = self.get_mesh_pose(part_name)    
    pf = part_pose.header.frame_id
    pp = part_pose.pose.position
    po = part_pose.pose.orientation

    (r,p,y) = euler_from_quaternion([po.x,po.y,po.z,po.w])
    go_list = quaternion_from_euler(r+m.pi,p,y+m.pi/2)
    gp_list = [pp.x, pp.y, pp.z]    

    br.sendTransform(gp_list, go_list, rospy.Time.now(), part_name+'_grasp', pf)

  def add_mesh(self, file_name):
    mesh_pose = geometry_msgs.msg.PoseStamped()

    mesh_pose.header.frame_id = "table"

    temp_r = random.uniform(-3.14,3.14)
    orientation_list = quaternion_from_euler(0,0,temp_r)
    mesh_pose.pose.position.x = random.uniform(-0.2, 0.2)
    # mesh_pose.pose.position.y = random.uniform(0, 0.4)
    mesh_pose.pose.position.y = random.uniform(-0.05, 0.05)
    mesh_pose.pose.position.z = 0.03
    position_list = [mesh_pose.pose.position.x, mesh_pose.pose.position.y, mesh_pose.pose.position.z]
    mesh_pose.pose.orientation.x = orientation_list[0]
    mesh_pose.pose.orientation.y = orientation_list[1]
    mesh_pose.pose.orientation.z = orientation_list[2]
    mesh_pose.pose.orientation.w = orientation_list[3]
    print mesh_pose.pose

    self.scene.add_mesh(self.part_name, mesh_pose, file_name, size=(1, 1, 1))

    print "[INFO] MESH NAME = '",self.part_name,"' ADDED"

  def rem_mesh(self):
    self.scene.remove_world_object(self.part_name)
    print "[INFO] MESH NAME = '",self.part_name,"' REMOVED"

  def __del__(self):
    self.scene.remove_world_object(self.part_name)