
import sys
import copy
import random
import roslib
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

from math import pi, sqrt, cos, sin
from numpy.linalg import inv as Inv
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from nav_msgs.msg import Odometry
from tf.transformations import*
from tf import*

from part_info import*          # part_file_address, part_name
from publish_sphere import*

def send_TF(pose,tf_name = "TF"):
	br = TransformBroadcaster()
	p_list = make_position_list(pose.position)
	o_list = make_orientation_list(pose.orientation)
	br.sendTransform(p_list, o_list, rospy.Time.now(), tf_name, 'world')
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
def Tmat_from_pose(pose):
	if not type(pose) is geometry_msgs.msg._Pose.Pose:
		print type(pose)
		print "TYPE ERROR"
	else:
		o_list = make_orientation_list(pose.orientation)
		Tmat = quaternion_matrix(o_list)
		Tmat[0,3] = pose.position.x
		Tmat[1,3] = pose.position.y
		Tmat[2,3] = pose.position.z
		return Tmat
def pose_from_Tmat(Tmat):
	pose = geometry_msgs.msg.PoseStamped().pose
	pose.position.x = Tmat[0,3]
	pose.position.y = Tmat[1,3]
	pose.position.z = Tmat[2,3]
	rpy = euler_from_matrix(Tmat[:3,:3])
	o_list = quaternion_from_euler(rpy[0],rpy[1],rpy[2])
	pose.orientation.x = o_list[0]
	pose.orientation.y = o_list[1]
	pose.orientation.z = o_list[2]
	pose.orientation.w = o_list[3]

	return pose

def make_circle_cmd(origin_point,axis_vector,init_point):
	th_list = np.arange(0,2*pi,pi/10)
	trace_list = []
	def unit(vector):
			norm = np.linalg.norm(vector)
			unit_vector = copy.deepcopy(vector/norm)
			return unit_vector
	ax_i = np.array(init_point) - np.array(origin_point)
	ax_k = unit(axis_vector)
	ax_j = np.cross(ax_k,ax_i)

	ax_mat = np.column_stack([ax_i,ax_j,ax_k])

	for th in th_list:
		new_point = np.dot(ax_i,cos(th)) + np.dot(ax_j,sin(th))
		trace_list.append(np.array(origin_point)+new_point)
	return trace_list
def make_spiral_cmd(initial):
        # spiral motion
        R = 0.006 #0.003
        revolution = 4 #4
        dth = pi/10
        th_len_1 = int(revolution*2*pi/dth) # len(th_array)-1
        dr = R/th_len_1 # R*0.005

        th_array = np.arange(0, revolution*2*pi+dth, step=dth)    
        r_array = np.arange(0, R+dr, step=dr)    
        th_array = np.concatenate((th_array, np.arange(dth, 2*pi+dth, step=dth)), axis=None)
        r_array = np.concatenate((r_array, np.ones(th_len_1/revolution)*R), axis=None)

        # print th_array

        spiral_list = []
        spiral_list.append(initial)
        i = 0
        for th in th_array:
            current = copy.deepcopy(initial)
            # current[0] += R*m.cos(th) - R 
            # current[1] += R*m.sin(th)
            current[0] += r_array[i]*cos(th) 
            current[1] += r_array[i]*sin(th)
            spiral_list.append(current)
            i += 1
        return spiral_list
class UR5(object):
	def __init__(self,G_name = 'rob1_arm'):
	    super(UR5, self).__init__()
	    moveit_commander.roscpp_initialize(sys.argv)
	    # rospy.init_node('ulol_UR5', anonymous=True)

	    self.robot = moveit_commander.RobotCommander()
	    self.scene = moveit_commander.PlanningSceneInterface()

	    group_names = self.robot.get_group_names()

	    self.group_name = G_name
	    
	    self.real_base_name = G_name.split('_')[0]+'_real_base_link'

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

	def move_cartesian_path(self, trans):
		pose = geometry_msgs.msg.PoseStamped().pose
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

	def get_toolpose_from_base(self,input_axis,tool_rpy,base_rpy):
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

	def move_tool_trace(self,trace_list,dir_axis,axis=[0,0,1]):
		#axis : axis of spiral, dir_axis : -y or z, trace_list is based on rob_base_link
		def unit(vector):
			norm = np.linalg.norm(vector)
			return vector/norm

		axis = unit(axis)
		waypoint = []
		sphere_pub_list = []
		print "CREATING LIST_OF_SPHERES"

		Iteration = len(trace_list)
		for i in range(Iteration):
			world_target = self.change_pose_cor_to_world(trace_list[i]
														,self.real_base_name)
			sphere_pub_list.append(copy.deepcopy(world_target))

			if dir_axis == 'z':
				tool_k = unit(np.dot(axis,-1))
				tool_i = unit(np.cross(trace_list[i],tool_k))
				tool_j = np.cross(tool_k,tool_i)
				tool_offset_vec = [0,0,-0.16]
				
			elif dir_axis == '-y':
				tool_j = axis
				tool_i = unit(np.cross(tool_j,trace_list[i]))
				tool_k = np.cross(tool_i,tool_j)
				tool_offset_vec = [0,0,-0.16]
			else:
				print "ERROR"
				break

			rotation = np.column_stack([tool_i,tool_j,tool_k]) # 3x3 matrix
			rot_list = euler_from_matrix(rotation) # [rol,pitch,yaw] of eef from base
			
			tool_offest = np.dot(rotation,tool_offset_vec)
			eef_point = np.array(trace_list[i])+np.array(tool_offest)
			eef_pose = pose_from_transrot(eef_point,rot_list)

			world_pose = self.change_pose_cor_to_world(eef_pose
													,self.real_base_name)
			waypoint.append(copy.deepcopy(world_pose))
			if (i+1)%3 == 0:
				print i+1,"/",Iteration


		print "DONE"
		return (waypoint,sphere_pub_list)


	def change_pose_cordinate(self,pose,prev_cord,new_cord):
		new_pose = self.move_group.get_current_pose(new_cord).pose
		prev_pose = self.move_group.get_current_pose(prev_cord).pose
		
		new_T = Tmat_from_pose(new_pose)
		prev_T = Tmat_from_pose(prev_pose)

		TF = prev_T.dot(Inv(new_T))

		prev = Tmat_from_pose(pose) #

		new = TF.dot(prev)
		new_pose = pose_from_Tmat(new)
		# send_TF(new_pose)
		return new_pose
	def change_point_cordinate(self,prev_point,prev_cord,new_cord):
		new_pose = self.move_group.get_current_pose(new_cord).pose
		prev_pose = self.move_group.get_current_pose(prev_cord).pose
		new_T = Tmat_from_pose(new_pose)
		prev_T = Tmat_from_pose(prev_pose)

		TF = prev_T.dot(Inv(new_T))
		point = np.append(prev_point,1)
		new_point = TF.dot(point)

		return new_point[0:3]
	def change_pose_cor_to_world(self,prev_pose,prev_cord):
		if type(prev_pose) == list or type(prev_pose) == tuple:
			prev = self.move_group.get_current_pose(prev_cord).pose
			TF = Tmat_from_pose(prev)
			point = np.append(prev_pose,1)
			new_point = TF.dot(point)
			return new_point[0:3]
		elif type(prev_pose) == geometry_msgs.msg._Pose.Pose:
			prev = self.move_group.get_current_pose(prev_cord).pose
			TF = Tmat_from_pose(prev)
			prev_mat = Tmat_from_pose(prev_pose)			
			new = TF.dot(prev_mat)
			new_pose = pose_from_Tmat(new)			 
			return new_pose
		else:
			print type(prev_pose)


def test_get_toolpose_from_base():
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
        joint_goal = [-pi/2,-pi/2-pi/4,-pi/2,pi/4,pi/2,pi]
        R2.go_to_joint_goal(joint_goal)

        tool_rpy = R2.move_group.get_current_rpy(R2.eef_link)
        base_rpy = R2.move_group.get_current_rpy(R2.real_base_name)
        R2.get_toolpose_from_base(['x','y'],tool_rpy,base_rpy)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
def test_move_tool_trace():
	
	rospy.init_node('ulol_UR5', anonymous=True)
	R2 = UR5('rob2_arm')
	R1 = UR5()
	R1.go_to_joint_goal([pi/2,-pi/2,pi/2,-pi/2,-pi/2,pi])
	R2.go_to_joint_goal([pi/2,-pi/2,pi/2,0,0,0])

	axis = [0,0,1]

	spiral_initial1 = [random.uniform(-0.2,0),random.uniform(-0.9,-0.7),random.uniform(0.04,0.06)]
	spiral1 = make_spiral_cmd(spiral_initial1)
	(waypoint1,sphere_pub_list1) = R1.move_tool_trace(spiral1,'z',axis)
	
	spiral_initial2 = [random.uniform(-0.2,0),random.uniform(-0.9,-0.7),random.uniform(0.04,0.06)]
	spiral2 = make_spiral_cmd(spiral_initial2)
	(waypoint2,sphere_pub_list2) = R2.move_tool_trace(spiral2,'-y',axis)

	print "READY"
	while True:	
		try:
			pub_sphere_list(sphere_pub_list1)
			(plan1, fraction1) = R1.move_group.compute_cartesian_path(waypoint1,0.01,0.0)
			R1.move_group.execute(plan1,wait = True)
			raw_input()
			R1.go_to_joint_goal([pi/2,-pi/2,pi/2,-pi/2,-pi/2,pi])

			pub_sphere_list(sphere_pub_list2)
			(plan2, fraction2) = R2.move_group.compute_cartesian_path(waypoint2,0.01,0.0)
			R2.move_group.execute(plan2,wait = True)
			raw_input()
			R2.go_to_joint_goal([pi/2,-pi/2,pi/2,0,0,0])

		except rospy.ROSInterruptException:
			return
		except KeyboardInterrupt:
			return

if __name__ == '__main__':
	test_move_tool_trace()
	# test_get_toolpose_from_base()
