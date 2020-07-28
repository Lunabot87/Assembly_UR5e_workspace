import tf
import rospy
import random
import moveit_commander
import geometry_msgs.msg

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

part_address = "/home/kdh/assembly_chair_stl/"
part1 = part_address+"chair_part1.SLDPRT.STL"
part2 = part_address+"chair_part2.SLDPRT.STL"
part3 = part_address+"chair_part3.SLDPRT.STL"
part4 = part_address+"chair_part4.SLDPRT.STL"
part5 = part_address+"chair_part5.SLDPRT.STL"
part6 = part_address+"chair_part6.SLDPRT.STL"




class Parts(object):
  def __init__(self, part_name):
    super(Parts, self).__init__()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.part_name = part_name
    self.xyz_length = [0.34,0.05,0.019]
    br = tf.TransformBroadcaster()

  def get_mesh_pose(self, mesh_name):
    mesh_pose = geometry_msgs.msg.PoseStamped()
    mesh_pose.header = self.scene.get_objects([mesh_name])[mesh_name].header
    mesh_pose.pose = self.scene.get_objects([mesh_name])[mesh_name].mesh_poses[0]
    return mesh_pose

  def add_mesh(self, file_name):
    mesh_pose = geometry_msgs.msg.PoseStamped()

    mesh_pose.header.frame_id = "world"

    temp_r = random.uniform(-3.14,3.14)
    orientation_list = quaternion_from_euler(0,0,temp_r)
    mesh_pose.pose.position.x = random.uniform(-0.4,0.3)
    mesh_pose.pose.position.y = random.uniform(0, 0.4)
    mesh_pose.pose.position.z = 0.81
    position_list = [mesh_pose.pose.position.x, mesh_pose.pose.position.y, mesh_pose.pose.position.z]
    mesh_pose.pose.orientation.x = orientation_list[0]
    mesh_pose.pose.orientation.y = orientation_list[1]
    mesh_pose.pose.orientation.z = orientation_list[2]
    mesh_pose.pose.orientation.w = orientation_list[3]
    # print mesh_pose.pose

    self.scene.add_mesh(self.part_name, mesh_pose, file_name, size=(1, 1, 1))
    br.sendTransform(position_list, orientation_list, rospy.Time.now(), self.part_name, 'world')

    print "[INFO] MESH NAME = '",self.part_name,"' ADDED"

  def rem_mesh(self):
    self.scene.remove_world_object(self.part_name)
    print "[INFO] MESH NAME = '",mesh_name,"' REMOVED"

  def __del__(self):
    self.scene.remove_world_object(self.part_name)