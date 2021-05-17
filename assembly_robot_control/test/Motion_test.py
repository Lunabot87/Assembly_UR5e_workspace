#!/usr/bin/env python
#-*- coding:utf-8 -*-
# import moveit_commander
# import moveit_msgs.msg
import rospy
from Assembly_Urx_test import UrxMotion
from Assembly_Math_test import *
from move_group_wrapper_test import MoveGroupCommanderWrapper
from std_srvs.srv import *
from sensor_msgs.msg import *
from ur_dashboard_msgs.srv import *
from pin_grasp_pose import *
from ASP_pose import *


from assembly_robot_msgs.srv import *
from assembly_robot_msgs.msg import *

from utils.conversions import *

import time

import copy

class Motion_test():
    def __init__(self):



        rospy.init_node('Motion_test', anonymous=True)

        # self.mg_rob1 = MoveGroupCommanderWrapper('rob1_arm', 'rob1_real_ee_link')
        # self.mg_rob2 = MoveGroupCommanderWrapper('rob2_arm', 'rob2_real_ee_link')
        
        # self.mg_rob1.set_planner_id("RRTConnectkConfigDefault")
        # self.mg_rob2.set_planner_id("RRTConnectkConfigDefault")
        
        self.urx_rob1 = UrxMotion("192.168.13.101")
        self.urx_rob2 = UrxMotion("192.168.13.100")

        self.rob1_client = rospy.ServiceProxy('/rob1/ur_hardware_interface/dashboard/play', Trigger)
        self.rob2_client = rospy.ServiceProxy('/rob2/ur_hardware_interface/dashboard/play', Trigger)

        self.rob1_check = rospy.ServiceProxy('/rob1/ur_hardware_interface/dashboard/program_running', IsProgramRunning)
        self.rob2_check = rospy.ServiceProxy('/rob2/ur_hardware_interface/dashboard/program_running', IsProgramRunning)

        self.rob_hand = rospy.Publisher('/hand/joint_states', JointState, queue_size=10)
      
        # self.program_running()

        # self.init_pose()

        self.rospy = rospy
        # time = ros.Time()

    def elp_camera_client(self, name, robot):
        rob_cam = ''
        if robot is False:
            rob_cam = 'camera_server_1'
        else:
            rob_cam = 'camera_server_2'

        for count in range(10):

            # self.rospy.wait_for_service(rob_cam)
            try:
                rob_client = self.rospy.ServiceProxy(rob_cam, cam_Srv)
                basler_data = rob_client(name)
                # print basler_client
                return basler_data
                break
            except self.rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        return False


    def client(self, hole_name, robot, tool = False):


        if robot is False:
            urx_rob = self.urx_rob1

        else:
            urx_rob = self.urx_rob2
        
        for count in range(10):

            result = self.elp_camera_client(hole_name, robot)

            if result is False:
                print "not found"
                break
                return False
                

            else:
                if tool is False:
                    time.sleep(0.2)

                    pose = urx_rob.robot.getl()
                    
                    pose[0] += result.x 
                    pose[1] += result.y - 0.0721

                    return pose
                else:
                    time.sleep(0.2)

                    pose = urx_rob.robot.getl()
                    
                    pose[0] += result.x 
                    pose[1] += result.y + 0.0719

                    return pose


        return [0,0,0,0,0,0]

    def program_running(self):
        rob1_connect = self.rob1_check()
        rob2_connect = self.rob2_check()
        while rob1_connect.program_running is not True:
            self.rob1_client()
            time.sleep(0.5)
            rob1_connect = self.rob1_check()
            if rob1_connect.program_running is True:
                break

        while rob2_connect.program_running is not True:
            self.rob2_client()
            time.sleep(0.5)
            rob2_connect = self.rob2_check()
            if rob2_connect.program_running is True:
                break

        time.sleep(2)

    def init_pose(self, robot=None):
        rob1_init_pose = self.mg_rob1.get_named_target_values("rob1_init_pose")
        rob2_init_pose = self.mg_rob2.get_named_target_values("rob2_init_pose")

        if robot is False:
            self.mg_rob1.go(rob1_init_pose)
        elif robot is True:
            self.mg_rob2.go(rob2_init_pose)
        else:
            self.mg_rob1.go(rob1_init_pose)
            self.mg_rob2.go(rob2_init_pose)

    def pin_grab_up(self, urx_rob):
        cur = urx_rob.robot.getl()

        force_mod = [1,1,1,0,0,0]
        force_toq = [0,0,20,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.05)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        urx_rob.robot.send_program(cmd_str)

        while True:
            print urx_rob.robot.getl()[2] - cur[2]
            if urx_rob.robot.getl()[2] - cur[2] > 0.02:
                urx_rob.robot.send_program("end_force_mode()")
                break


    def part5_insert_motion(self):


        self.urx_rob1.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4, wait=False)
        self.urx_rob2.robot.movej(rob2_init_pose, vel = 1.05, acc = 1.4)

        

        self.urx_rob1.robot.movej(part6_pose1_1, vel = 1.05, acc = 1.4, wait=False)

        self.urx_rob2.robot.movej(part5_grap_pre, vel = 1.05, acc = 1.4)

        self.urx_rob1.robot.movej(part6_pose1_2, vel = 1.05, acc = 1.4, wait=False)

        cur = self.urx_rob2.robot.getl()
        cur[2] -= 0.4
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(255)

        cur[2] += 0.4
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25, wait=False)

        cur = self.urx_rob1.robot.getl()
        cur[0] -= 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)


        ###############################

        ###### part6 insert ##########
        

        self.urx_rob1.gripper_move_and_wait(255)

        
        
        self.urx_rob2.robot.movej(waypoint1, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(waypoint2, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(waypoint3, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(waypoint4, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(goal1, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(goal2, vel = 1.05, acc = 1.4)

        self.urx_rob2.robot.send_program("zero_ftsensor()")
        time.sleep(0.1)

        force_mod = [0,0,1,1,0,0]
        force_toq = [0,0,-15,0,0,0] 

        # print "down?"
        # raw_input()

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.05)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        while(True):
            try:
                force = self.urx_rob2.robot.get_tcp_force()

                if force[2] > 10:
                    self.urx_rob2.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.urx_rob2.robot.send_program("end_force_mode()")
                break

        self.urx_rob2.spiral_motion()


        force_mod = [1,0,1,1,1,0]
        force_toq = [0,0,-15,0,2,0] 


        # print "down?"
        # raw_input()

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.05)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        #######part6_part2_insert motion#########

        self.urx_rob1.spiral_motion()

        self.urx_rob1.gripper_move_and_wait(0)

        self.urx_rob1.robot.movej(part6_pose1_2, vel = 1.05, acc = 1.4)
        self.urx_rob1.robot.movej(part6_pose1_1, vel = 1.05, acc = 1.4)
        self.urx_rob1.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)


        self.urx_rob1.robot.movej(part6_hold1_rob1, vel = 1.05, acc = 1.4)

        cur = self.urx_rob1.robot.getl()
        cur[2] -= 0.1
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.gripper_move_and_wait(255)

        force_mod = [0,0,1,0,1,0]
        force_toq = [0,0,-15,0,-2,0] 

        # print "down?"
        # raw_input()

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.05)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)


        ################part4 insert motion####################

        self.urx_rob2.robot.send_program("end_force_mode()")

        self.urx_rob2.gripper_move_and_wait(0)


        self.urx_rob2.robot.movej(rob2_init_pose, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(part6_pose2_1, vel = 1.05, acc = 1.4)
        cur = self.urx_rob2.robot.getl()
        cur[2] -= 0.1
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(255)

        self.urx_rob2.spiral_motion()

        self.urx_rob1.gripper_move_and_wait(0)

        self.urx_rob1.robot.movej(part6_hold2_rob1, vel = 1.05, acc = 1.4)

        self.urx_rob1.gripper_move_and_wait(255)

        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-15,0,0,0] 

        # print "down?"
        # raw_input()

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)


        ###############part3 insert motion ########################

        self.urx_rob2.gripper_move_and_wait(0)
        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.1
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)
        self.urx_rob2.robot.movej(rob2_init_pose, vel = 1.05, acc = 1.4)


        ######################8min################################

        # self.urx_rob2.robot.movej(part6_pose3_1, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part6_pose3_2, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part6_pose3_3, vel = 1.05, acc = 1.4)

        # cur = self.urx_rob2.robot.getl()
        # cur[0] += 0.1
        # self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        # self.urx_rob2.gripper_move_and_wait(255)

        # self.urx_rob2.spiral_motion()

        # self.urx_rob2.gripper_move_and_wait(0)

        # self.urx_rob2.robot.movej(part6_pose3_3, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part6_pose3_2, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part6_pose3_1, vel = 1.05, acc = 1.4)


        # self.urx_rob2.robot.movej(rob2_init_pose, vel = 1.05, acc = 1.4)


        # self.urx_rob2.robot.movej(part6_pose2_1, vel = 1.05, acc = 1.4)
        # cur = self.urx_rob2.robot.getl()
        # cur[2] -= 0.1
        # self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        # # print "down?"
        # # raw_input()

        # force_mod = [0,0,1,0,0,0]
        # force_toq = [0,0,-50,0,0,0] 

        # cmd_str  = "def go_down():"
        # cmd_str += "\tforce_mode_set_damping(0.05)\n"
        # cmd_str += "\twhile (True):\n"
        # cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        # cmd_str += "\t\tsync()\n"
        # cmd_str += "\tend\n"
        # cmd_str += "end\n"

        # self.urx_rob2.robot.send_program(cmd_str)

        # time.sleep(3)

        # self.urx_rob2.robot.movej(part6_pose2_1, vel = 1.05, acc = 1.4)

        # self.urx_rob2.robot.movej(rob2_init_pose, vel = 1.05, acc = 1.4)


        #####################################################

        self.urx_rob1.gripper_move_and_wait(0)

        self.urx_rob1.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)

    def screw_motion(self, urx_rob):
        self.urx_rob1.robot.set_digital_out(7, False)
        self.urx_rob1.robot.set_digital_out(6, False)
        self.urx_rob1.robot.set_digital_out(5, False)
        time.sleep(0.1)
        #### slow screw ####
        self.urx_rob1.robot.set_digital_out(7, True)
        time.sleep(0.1)
        self.urx_rob1.robot.set_digital_out(5, True)
        ####################


        urx_rob.screw_motion()


        ##### screw init #########
        self.urx_rob1.robot.set_digital_out(5, False)
        time.sleep(0.1)
        self.urx_rob1.robot.set_digital_out(7, False)
        ##########################

        time.sleep(0.5)

        ######## fast screw ########
        self.urx_rob1.robot.set_digital_out(6, True)
        time.sleep(0.1)
        self.urx_rob1.robot.set_digital_out(5, True)
        ############################

        urx_rob.screw_motion_insert()

        while self.urx_rob1.robot.get_digital_in(5) is not True:
            time.sleep(0.1)
            # print self.robot.get_digital_in(5)
            if self.urx_rob1.robot.get_digital_in(5):
                break

        urx_rob.robot.set_digital_out(6, False)
        urx_rob.robot.set_digital_out(5, False)

        urx_rob.screw_motion_reverse()


    def long_screw_insert1(self):

        #######long screw############
        self.urx_rob1.robot.movej(screw_cam_pose_1, vel = 1.05, acc = 1.4)

        move_offset = self.client("hole5-8", False)

        print "move_offset\n{0}".format(move_offset)

        ####pin을 잡았다고 가정(수정 필요 필수)########
        self.urx_rob1.robot.movej(rob1_104322_pre_1, vel = 1.05, acc = 1.4)

        self.urx_rob1.gripper_move_and_wait(100)

        self.urx_rob1.robot.movej(rob1_104322_1, vel = 1.05, acc = 0.25)

        self.urx_rob1.gripper_move_and_wait(255, force = True)

        cur = self.urx_rob1.robot.getl()

        force_mod = [1,1,1,0,0,0]
        force_toq = [0,0,30,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.05)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)

        while True:
            # print self.urx_rob1.robot.getl()[2] - cur[2]
            if self.urx_rob1.robot.getl()[2] - cur[2] > 0.1:
                self.urx_rob1.robot.send_program("end_force_mode()")
                break

        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.3
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25, wait=False)
        
        #############################
        #pin insertion#

        ####hold####
        self.urx_rob2.robot.movej(screw_hold_pose_1, vel = 1.05, acc = 1.4)
        cur = self.urx_rob2.robot.getl()
        cur[2] -= 0.1
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(255)
        #############

        self.urx_rob1.robot.movej(screw_long_pre_1, vel = 1.05, acc = 1.4)

        self.urx_rob1.robot.movej(screw_long_1, vel = 1.05, acc = 1.4)

        cur = self.urx_rob1.robot.getl()
        cur[0] = move_offset[0]
        cur[1] = move_offset[1]
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25, wait = False)

        self.urx_rob1.spiral_motion()

        self.urx_rob1.screw_motion_insert()

        time.sleep(2)

        self.urx_rob1.gripper_move_and_wait(0)

        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.05
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        # self.urx_rob1.robot.movej(screw_cam_pose_1, vel = 1.05, acc = 1.4)

        # move_offset = self.client("hole5-8", False, tool=True)

        # print "move_offset\n{0}".format(move_offset)

        self.urx_rob1.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)

        ######
        ###tool connect#######


        self.urx_rob1.robot.movej(screw_long_tool_pre_1, vel = 1.05, acc = 1.4)

        cur = self.urx_rob1.robot.getl()
        cur[0] += 0.2

        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.gripper_move_and_wait(255)


        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.4
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        curj = self.urx_rob1.robot.getj()

        self.urx_rob1.robot.movej(screw_long_tool_1, vel = 1.05, acc = 1.4)

        cur = self.urx_rob1.robot.getl()
        cur[0] = move_offset[0]
        cur[1] = (move_offset[1] + 0.154)
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.screw_motion(self.urx_rob1)


        ##############초기화#############

        self.urx_rob1.robot.movej(curj, vel = 1.05, acc = 0.25)

        self.urx_rob1.robot.movej(screw_long_tool_pre_1, vel = 1.05, acc = 1.4)

        cur = self.urx_rob1.robot.getl()

        cur[0] += 0.2

        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.gripper_move_and_wait(0)

        self.urx_rob1.robot.movej(screw_long_tool_pre_1, vel = 1.05, acc = 1.4)

        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.4
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.tool_station(True)

        self.urx_rob2.robot.send_program("end_force_mode()")

        self.urx_rob2.gripper_move_and_wait(0)

        
        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.1
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)
        self.urx_rob1.robot.movej(rob2_init_pose, vel = 1.05, acc = 1.4)

        time.sleep(10)


    def long_screw_insert2(self):

        #######long screw############

        self.urx_rob2.robot.movej(screw_cam_pose_2, vel = 1.05, acc = 1.4)

        move_offset = self.client("hole5-9", True)

        print "move_offset\n{0}".format(move_offset)

        ####pin을 잡았다고 가정(수정 필요 필수)########
        self.urx_rob2.robot.movej(rob2_104322_pre_3, vel = 1.05, acc = 1.4)

        self.urx_rob2.gripper_move_and_wait(100)

        self.urx_rob2.robot.movej(rob2_104322_3, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(255, force = True)

        cur = self.urx_rob2.robot.getl()

        force_mod = [1,1,1,0,0,0]
        force_toq = [0,0,30,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.05)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        while True:
            
            if self.urx_rob2.robot.getl()[2] - cur[2] > 0.1:
                self.urx_rob2.robot.send_program("end_force_mode()")
                break

        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.3
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)
        
        #############################
        #pin insertion#

        ####hold####
        self.urx_rob1.robot.movej(part6_hold1_rob1, vel = 1.05, acc = 1.4)
        cur = self.urx_rob1.robot.getl()
        cur[2] -= 0.1
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.gripper_move_and_wait(255)
        ####hold####

        self.urx_rob2.robot.movej(screw_long_pre_2, vel = 1.05, acc = 1.4)

        self.urx_rob2.robot.movej(screw_long_2, vel = 1.05, acc = 1.4)



        # cur = self.urx_rob2.robot.getl()
        # cur[0] = move_offset[0]
        # cur[1] = move_offset[1]
        # self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25, wait = False)

        curp = self.urx_rob2.robot.getl()

        self.urx_rob2.spiral_motion()

        self.urx_rob2.screw_motion_insert()

        time.sleep(2)

        self.urx_rob2.gripper_move_and_wait(0)

        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.05
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        curj = self.urx_rob2.robot.getl()

        self.urx_rob2.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)

        ######
        ###tool connect#######


        self.urx_rob2.robot.movej(screw_long_tool_pre_2, vel = 1.05, acc = 1.4)

        cur = self.urx_rob2.robot.getl()
        cur[0] -= 0.2

        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(255)


        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.4
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        curj = self.urx_rob2.robot.getj()

        self.urx_rob2.robot.movej(screw_long_tool_2, vel = 1.05, acc = 1.4)

        # cur = self.urx_rob2.robot.getl()
        # cur[0] += (curp[0] - curj[0])
        # cur[1] += (curp[0] - curj[1])
        # self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.screw_motion(self.urx_rob2)


        ##############초기화#############

        self.urx_rob2.robot.movej(curj, vel = 1.05, acc = 0.25)

        self.urx_rob2.robot.movej(screw_long_tool_pre_2, vel = 1.05, acc = 1.4)

        cur = self.urx_rob2.robot.getl()

        cur[0] -= 0.2

        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(0)

        self.urx_rob2.robot.movej(screw_long_tool_pre_2, vel = 1.05, acc = 1.4)

        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.4
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)


        self.urx_rob1.robot.send_program("end_force_mode()")

        self.urx_rob1.gripper_move_and_wait(0)

        
        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.1
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)
        self.urx_rob1.robot.movej(rob2_init_pose, vel = 1.05, acc = 1.4)


    def long_screw_insert3(self):

        #######long screw############

        ####pin을 잡았다고 가정(수정 필요 필수)########
        self.urx_rob2.robot.movej(rob2_104322_pre_5, vel = 1.05, acc = 1.4)

        self.urx_rob2.gripper_move_and_wait(100)

        self.urx_rob2.robot.movej(rob2_104322_5, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(255, force = True)

        cur = self.urx_rob2.robot.getl()

        force_mod = [1,1,1,0,0,0]
        force_toq = [0,0,30,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.05)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        while True:
            print self.urx_rob2.robot.getl()[2] - cur[2]
            if self.urx_rob2.robot.getl()[2] - cur[2] > 0.1:
                self.urx_rob2.robot.send_program("end_force_mode()")
                break

        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.3
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)
        
        #############################
        #pin insertion#

        ####hold####
        self.urx_rob1._gripper_move_t(120)
        self.urx_rob1.robot.movej(part4_hold_pre, vel = 3, acc = 2)
        
        cur = self.urx_rob1.robot.getl()
        cur[1] -= 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)

        self.urx_rob1.gripper_move_and_wait(255)

        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-30,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)
        ####hold####

        self.urx_rob2.robot.movej(screw_long_pre_3, vel = 1.05, acc = 1.4)

        self.urx_rob2.robot.movej(screw_long_3, vel = 1.05, acc = 0.4)

        # print "stop!!!"
        # raw_input()

        curp = self.urx_rob2.robot.getl()
        
        self.urx_rob2.spiral_motion()

        self.urx_rob2.screw_motion_insert()

        time.sleep(2)

        self.urx_rob2.gripper_move_and_wait(0)

        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.05
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        curj = self.urx_rob2.robot.getl()

        self.urx_rob2.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)

        ######
        ###tool connect#######


        self.urx_rob2.robot.movej(screw_long_tool_pre_3, vel = 1.05, acc = 1.4)

        cur = self.urx_rob2.robot.getl()
        cur[0] -= 0.2

        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(255)


        # print "stop!!!"
        # raw_input()


        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.4
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        curj = self.urx_rob2.robot.getj()

        self.urx_rob2.robot.movej(screw_long_tool_3, vel = 1.05, acc = 1.4)

        # cur = self.urx_rob2.robot.getl()
        # cur[0] += (curp[0] - curj[0])
        # cur[1] += (curp[0] - curj[1])
        # self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.screw_motion(self.urx_rob2)


        ##############초기화#############

        self.urx_rob2.robot.movej(curj, vel = 1.05, acc = 0.25)

        self.urx_rob2.robot.movej(screw_long_tool_pre_3, vel = 1.05, acc = 1.4)

        cur = self.urx_rob2.robot.getl()

        cur[0] -= 0.2

        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(0)

        self.urx_rob2.robot.movej(screw_long_tool_pre_3, vel = 1.05, acc = 1.4)

        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.4
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.tool_station(False)

        self.urx_rob1.robot.send_program("end_force_mode()")
        
        self.urx_rob1.gripper_move_and_wait(120)
        cur = self.urx_rob1.robot.getl()
        cur[1] += 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob1.gripper_move_and_wait(0)

        self.urx_rob2.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)
        self.urx_rob1.robot.movej(rob2_init_pose, vel = 1.05, acc = 1.4)

    def tool_station(self, rob = False):
        self.urx_rob2.robot.set_digital_out(0, False)
        self.urx_rob2.robot.set_digital_out(1, False)
        self.urx_rob2.robot.set_digital_out(2, False)
        self.urx_rob2.robot.set_digital_out(3, False)

        if rob is False:
            self.urx_rob2.robot.set_digital_out(0, True)
            self.urx_rob2.robot.set_digital_out(1, False)
        else:
            self.urx_rob2.robot.set_digital_out(0, False)
            self.urx_rob2.robot.set_digital_out(1, True)
        
        self.urx_rob2.robot.set_digital_out(2, False)
        self.urx_rob2.robot.set_digital_out(3, False)

    def wood_pin_motion(self, urx_rob, pre_pin, pin, pose, tool):

        urx_rob._gripper_move_t(170)

        urx_rob.robot.movej(pre_pin, vel = 3, acc = 2)

        urx_rob.robot.movej(pin, vel = 3, acc = 2)

        urx_rob.gripper_move_and_wait(255, force = True)

        self.pin_grab_up(urx_rob)

        self.tool_station(tool)

        cur = urx_rob.robot.getl()
        cur[2] += 0.2
        urx_rob.robot.movel(cur, vel = 1.57, acc = 1.57)

        urx_rob.robot.movej(pose, vel = 3, acc = 2)

        urx_rob.spiral_motion()

        urx_rob._gripper_move_t(0)

        time.sleep(0.2)

        cur = urx_rob.robot.getl()
        cur[2] += 0.1
        urx_rob.robot.movel(cur, vel = 1.57, acc = 1.57)

    def base_wood_pin_insert(self):
        
        self.tool_station(False)
        self.urx_rob1.robot.movej(rob1_init_pose, vel = 1.57, acc = 1.57)
        self.urx_rob2.robot.movej(rob2_init_pose, vel = 1.57, acc = 1.57)


        self.wood_pin_motion(self.urx_rob1, rob1_101350_pre_1, rob1_101350_1, part5_hole1, tool = False)

        # print "go?"
        # raw_input()

        self.wood_pin_motion(self.urx_rob1, rob1_101350_pre_2, rob1_101350_2, part5_hole2, tool = True)

        print "move station"
        # raw_input()

        self.urx_rob1.robot.movej(rob1_init_pose, vel = 3, acc = 2, wait=False)

        time.sleep(14)

        self.wood_pin_motion(self.urx_rob2, rob2_101350_pre_3, rob2_101350_3, part5_hole3, tool = True)

        self.wood_pin_motion(self.urx_rob2, rob2_101350_pre_4, rob2_101350_4, part5_hole4, tool = True)

        self.wood_pin_motion(self.urx_rob2, rob2_101350_pre_5, rob2_101350_5, part5_hole5, tool = True)

        self.wood_pin_motion(self.urx_rob2, rob2_101350_pre_6, rob2_101350_6, part5_hole6, tool = True)

        self.wood_pin_motion(self.urx_rob2, rob2_101350_pre_7, rob2_101350_7, part5_hole7, tool = True)
        
        # print "move station"
        # raw_input()

        self.urx_rob2.robot.movej(rob2_101350_pre_10, vel = 3, acc = 2, wait=False)

        ###########

    def part2_wood_pin_insert(self):

        self.wood_pin_motion(self.urx_rob2, rob2_101350_pre_10, rob2_101350_10, part2_hole3, tool = True)

        self.wood_pin_motion(self.urx_rob2, rob2_101350_pre_11, rob2_101350_11, part2_hole4, tool = False)

        self.urx_rob2.robot.movej(rob2_init_pose, vel = 3, acc = 2, wait = False)

    def part2_insert_motion(self):

        # self.urx_rob2.robot.movej(part2_hold_pre_2)
        # cur = self.urx_rob2.robot.getl()
        # cur[2] -= 0.2
        # self.urx_rob2.robot.movel(cur)

        ##### part2 insert ######
        self.urx_rob1.robot.movej(part2_grap_pre, vel = 3, acc = 2)
        cur = self.urx_rob1.robot.getl()
        cur[2] -= 0.2
        self.urx_rob1.robot.movel(cur, vel = 3, acc = 2)
        self.urx_rob1.gripper_move_and_wait(255)
        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob1.robot.movej(part2_rotate, vel = 3, acc = 2)
        self.urx_rob1.robot.movej(part2_insert, vel = 3, acc = 2)
        self.urx_rob1.spiral_motion()

        self.urx_rob1.robot.send_program("end_force_mode()")

        self.part2_wood_pin_insert()

        ####gripper open#####
        self.urx_rob1.gripper_move_and_wait(0)

        #init pose##

        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.25
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob1.robot.movej(rob1_101350_pre_8, vel = 3, acc = 2, wait=False)

        time.sleep(2)

        ############################

    def part3_wood_pin_insert(self):

        self.wood_pin_motion(self.urx_rob1, rob1_101350_pre_8, rob1_101350_8, part3_hole3, tool = False)

        self.wood_pin_motion(self.urx_rob1, rob1_101350_pre_9, rob1_101350_9, part3_hole4, tool = True)

        self.urx_rob1.robot.movej(part4_move_grap, vel = 3, acc = 2, wait = False)

    def part3_insert_motion(self):

        ##### part3 insert ######

        self.urx_rob2.robot.movej(part3_grap_pre, vel = 3, acc = 2)

        cur = self.urx_rob2.robot.getl()
        cur[2] -= 0.2
        self.urx_rob2.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob2.gripper_move_and_wait(255)
        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.2
        self.urx_rob2.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob2.robot.movej(part3_rotate, vel = 3, acc = 2)
        self.urx_rob2.robot.movej(part3_insert, vel = 3, acc = 2)
        self.urx_rob2.spiral_motion()

        self.urx_rob2.robot.send_program("end_force_mode()")

        self.part3_wood_pin_insert()

        ####gripper open#####
        self.urx_rob2.gripper_move_and_wait(0)

        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.25
        self.urx_rob2.robot.movel(cur, vel = 1.57, acc = 1.57)

        self.urx_rob2.robot.movej(rob2_init_pose, vel = 3, acc = 2, wait=True)

        time.sleep(2)

    def part4_insert_motion(self):

        #### part4 move #####

        self.urx_rob1._gripper_move_t(120)
        self.urx_rob1.robot.movej(part4_move_grap, vel = 3, acc = 2)
        
        cur = self.urx_rob1.robot.getl()
        cur[2] -= 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob1.gripper_move_and_wait(255)
        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob1.robot.movej(part4_move_point, vel = 3, acc = 2, wait=False)


        ###### part4 insert ######

        self.urx_rob2.robot.movej(part4_grap_pre, vel = 3, acc = 2)

        cur = self.urx_rob2.robot.getl()
        cur[1] -= 0.2
        self.urx_rob2.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob2.gripper_move_and_wait(255)

        self.urx_rob1._gripper_move_t(120)
        cur = self.urx_rob1.robot.getl()
        cur[2] -= 0.1
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)

        cur = self.urx_rob2.robot.getl()
        cur[1] += 0.2
        self.urx_rob2.robot.movel(cur, vel = 1.57, acc = 1.57)


        self.urx_rob2.robot.movej(part4_rotate, vel = 3, acc = 2)
        self.urx_rob2.robot.movej(part4_insert, vel = 3, acc = 2)

        self.urx_rob2.spiral_motion()

        self.urx_rob2.gripper_move_and_wait(0)
        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.2
        self.urx_rob2.robot.movel(cur, vel = 1.57, acc = 1.57, wait=False)

        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob1.robot.movej(rob1_init_pose, vel = 3, acc = 2, wait=False)
        self.urx_rob2.robot.movej(rob2_init_pose, vel = 3, acc = 2)

        ###########################


    def part4_wood_pin_insert(self):

        ###### part4 wood pin hold ######
        self.urx_rob1._gripper_move_t(120)
        self.urx_rob1.robot.movej(part4_hold_pre, vel = 3, acc = 2)
        
        cur = self.urx_rob1.robot.getl()
        cur[1] -= 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)

        self.urx_rob1.gripper_move_and_wait(255)

        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-30,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)

        self.wood_pin_motion(self.urx_rob2, rob2_101350_pre_12, rob2_101350_12, part4_hole1, tool = True)
        self.wood_pin_motion(self.urx_rob2, rob2_101350_pre_13, rob2_101350_13, part4_hole2, tool = True)
        self.wood_pin_motion(self.urx_rob2, rob2_101350_pre_14, rob2_101350_14, part4_hole3, tool = False)

        self.urx_rob2.robot.movej(rob2_init_pose, vel = 3, acc = 2, wait=False)

        self.urx_rob1.robot.send_program("end_force_mode()")

        self.urx_rob1.gripper_move_and_wait(120)
        cur = self.urx_rob1.robot.getl()
        cur[1] += 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob1.gripper_move_and_wait(0)

        self.urx_rob1.robot.movej(rob1_init_pose, vel = 3, acc = 2)

    def table_screw(self, mod):
        self.urx_rob2.robot.set_digital_out(0, False)
        self.urx_rob2.robot.set_digital_out(1, False)
        self.urx_rob2.robot.set_digital_out(2, False)
        self.urx_rob2.robot.set_digital_out(3, False)

        if mod is 1:
            self.urx_rob2.robot.set_digital_out(3, True)
            self.urx_rob2.robot.set_digital_out(2, True)
            self.urx_rob2.robot.set_digital_out(1, True)
        elif mod is 2:
            self.urx_rob2.robot.set_digital_out(3, True)
            self.urx_rob2.robot.set_digital_out(1, True)

        elif mod is 3:
            self.urx_rob2.robot.set_digital_out(2, True)
            self.urx_rob2.robot.set_digital_out(1, True)
        else:
            self.urx_rob2.robot.set_digital_out(0, False)
            self.urx_rob2.robot.set_digital_out(1, False)
            self.urx_rob2.robot.set_digital_out(2, False)
            self.urx_rob2.robot.set_digital_out(3, False)
            
    def table_screw_insert1(self):
        self.urx_rob1._gripper_move_t(120)
        self.urx_rob1.robot.movej(part4_hold_pre, vel = 3, acc = 2)
        
        cur = self.urx_rob1.robot.getl()
        cur[1] -= 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)

        self.urx_rob1.gripper_move_and_wait(255)

        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-30,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)

        self.table_screw(1)

        # time.sleep(40)

        # self.urx_rob1.robot.send_program("end_force_mode()")

        # self.urx_rob1.gripper_move_and_wait(120)
        # cur = self.urx_rob1.robot.getl()
        # cur[1] += 0.2
        # self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)
        # self.urx_rob1.gripper_move_and_wait(0)

        # self.urx_rob1.robot.movej(rob1_init_pose, vel = 3, acc = 2)

    def table_screw_insert2(self):

        self.urx_rob2.robot.movej(part6_pose3_1, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(part6_pose3_2, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(part6_pose3_3, vel = 1.05, acc = 1.4)

        cur = self.urx_rob2.robot.getl()
        cur[0] += 0.1
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(255)

        self.table_screw(2)

        time.sleep(50)

        # self.urx_rob2.gripper_move_and_wait(0)

        # self.urx_rob2.robot.movej(part6_pose3_3, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part6_pose3_2, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part6_pose3_1, vel = 1.05, acc = 1.4)
        # self.urx_rob2.gripper_move_and_wait(255)

        # self.urx_rob2.robot.movej(rob2_init_pose, vel = 3, acc = 2)



        # time.sleep(1)

        # self.urx_rob2.robot.set_digital_out(1,False)
        # self.urx_rob2.robot.set_digital_out(2,True)



        force_mod = [1,1,1,0,0,0]
        force_toq = [0,0,-20,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)

        self.urx_rob1.robot.set_digital_out(3,False)
        self.urx_rob1.robot.set_digital_out(2,False)

        time.sleep(10)  

        self.urx_rob1.robot.set_digital_out(2,True)
        self.urx_rob1.robot.set_digital_out(3,True)

        time.sleep(20)

        self.urx_rob1.robot.send_program("end_force_mode()")

        

        self.urx_rob1.gripper_move_and_wait(120)
        cur = self.urx_rob1.robot.getl()
        cur[1] += 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.57, acc = 1.57)
        self.urx_rob1.gripper_move_and_wait(0)

        self.urx_rob1.robot.movej(rob1_init_pose, vel = 3, acc = 2)

    def table_screw_insert3(self):

        self.urx_rob1.robot.movej(part6_pose1_1, vel = 1.05, acc = 1.4)

        self.urx_rob1.robot.movej(part6_pose1_2, vel = 1.05, acc = 1.4)

        cur = self.urx_rob1.robot.getl()
        cur[0] -= 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.gripper_move_and_wait(255)

        force_mod = [1,1,1,0,0,0]
        force_toq = [0,0,-20,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)

        self.table_screw(3)

        time.sleep(70)

        # self.urx_rob1.gripper_move_and_wait(0)

        # self.urx_rob1.robot.movej(part6_pose1_2, vel = 1.05, acc = 1.4)

        # self.urx_rob1.robot.movej(part6_pose1_1, vel = 1.05, acc = 1.4)

        # self.urx_rob1.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)


        # self.urx_rob1.robot.send_program("end_force_mode()")
        # self.urx_rob2.robot.send_program("end_force_mode()")

        # force_mod = [0,0,1,0,0,0]
        # force_toq = [0,0,40,0,0,0]

        # cmd_str  = "def go_down():"
        # cmd_str += "\tforce_mode_set_damping(0.00)\n"
        # cmd_str += "\twhile (True):\n"
        # cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        # cmd_str += "\t\tsync()\n"
        # cmd_str += "\tend\n"
        # cmd_str += "end\n"

        # # self.urx_rob2.robot.send_program(cmd_str)
        # self.urx_rob1.robot.send_program(cmd_str)
        
        # pose = self.urx_rob1.robot.getl()
        # while (self.urx_rob1.robot.getl()[2] - pose[2]) < 0.02:
        #     print "upupup!!! and a = {0}".format(self.urx_rob1.robot.getl()[2])
        #     time.sleep(0.5)


        self.urx_rob1.robot.send_program("end_force_mode()")
        self.urx_rob2.robot.send_program("end_force_mode()")

        cur1 = self.urx_rob1.robot.getl()
        cur2 = self.urx_rob2.robot.getl()

        cur1[2] += 0.02
        cur2[2] += 0.02

        self.urx_rob1.robot.movel(cur1, vel = 1.05, acc = 0.25, wait=False)
        self.urx_rob2.robot.movel(cur2, vel = 1.05, acc = 0.25)

        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-5,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)
        self.urx_rob2.robot.send_program(cmd_str)

        time.sleep(2)



        self.urx_rob1.gripper_move_and_wait(0)

        self.urx_rob1.robot.movej(part6_pose1_2, vel = 1.05, acc = 1.4)

        self.urx_rob1.robot.movej(part6_pose1_1, vel = 1.05, acc = 1.4)

        self.urx_rob1.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4, wait=False)



        # self.urx_rob2.gripper_move_and_wait(0)

        # self.urx_rob2.robot.movej(part6_pose3_3, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part6_pose3_2, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part6_pose3_1, vel = 1.05, acc = 1.4)

        # self.urx_rob2.robot.movej(rob2_init_pose, vel = 3, acc = 2)


        self.table_screw(4)


        self.urx_rob1.robot.set_digital_out(0,True)
        self.urx_rob1.robot.set_digital_out(1,True)

    def turn_motion(self):
        self.urx_rob1.robot.set_digital_out(0, True)
        self.urx_rob1.robot.set_digital_out(1, True)
        time.sleep(1)

        while True:
            if self.urx_rob1.robot.get_digital_in(0):
                break


        self.urx_rob1.robot.movej(turn_point1_rob1, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(turn_point1_rob2, vel = 1.05, acc = 1.4)


        self.urx_rob1.robot.movej(turn_point2_rob1, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(turn_point2_rob2, vel = 1.05, acc = 1.4)


        cur = self.urx_rob2.robot.getl()
        cur[1] -= 0.05
        self.urx_rob2.robot.movel(cur, vel = 0.5, acc = 0.25)


        self.urx_rob2.gripper_move_and_wait(255)

        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.02
        self.urx_rob2.robot.movel(cur, vel = 0.5, acc = 0.25)


        cur = self.urx_rob2.robot.getl()
        cur[1] -= 0.05
        self.urx_rob2.robot.movel(cur, vel = 0.5, acc = 0.25)

        self.urx_rob2.robot.send_program("zero_ftsensor()")

        force_mod = [0,1,0,0,0,0]
        force_toq = [0,-10,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        time.sleep(5)

        while(True):
            try:
                force = self.urx_rob2.robot.get_tcp_force()

                if abs(force[1]) > 10:
                    # print force[0], force[1]
                    self.urx_rob2.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.urx_rob2.robot.send_program("end_force_mode()")
                break

        self.urx_rob2.gripper_move_and_wait(255)

        force_mod = [0,1,0,0,0,0]
        force_toq = [0,-2,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        self.urx_rob1.gripper_move_and_wait(255)

        cur = self.urx_rob1.robot.getl()
        cur[1] -= 0.05
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.robot.set_digital_out(1, False)
        self.urx_rob1.robot.set_digital_out(0, False)
        time.sleep(1)

        while True:
            if self.urx_rob1.robot.get_digital_in(0):
                break

        self.urx_rob1.robot.set_digital_out(3, False)
        self.urx_rob1.robot.set_digital_out(2, False)

        time.sleep(1)

        while True:
            if self.urx_rob1.robot.get_digital_in(0):
                break

        self.urx_rob1.robot.set_digital_out(1, True)

        time.sleep(1)
        while True:
            if self.urx_rob1.robot.get_digital_in(0):
                break
        self.urx_rob1.robot.set_digital_out(3, True)

        time.sleep(1)

        while True:
            if self.urx_rob1.robot.get_digital_in(0):
                break

        force_mod = [1,1,1,0,1,0]
        force_toq = [0,0,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        # print "go? turn rob1"
        # raw_input()

        curj = self.urx_rob1.robot.getj()
        curj[5] = turn_point3_rob1[5]

        self.urx_rob1.robot.movej(curj)

        self.urx_rob1.robot.set_digital_out(0, True)
        time.sleep(3)

        self.urx_rob2.robot.send_program(cmd_str) 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tfreedrive_mode()\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"


        self.urx_rob2.robot.send_program(cmd_str) 
        self.urx_rob1.robot.send_program(cmd_str)

        while True:
            if self.urx_rob1.robot.get_digital_in(0):
                break

        self.urx_rob1.gripper_move_and_wait(0)
        self.urx_rob2.gripper_move_and_wait(0)

        #####################정렬####################################
        cur = self.urx_rob1.robot.getl()
        cur[1] += 0.17
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)


        self.urx_rob1.robot.movej(rob1_init_pose, vel = 1.5, acc = 1.4)


        self.urx_rob2.robot.movej(turn_point3_rob2, vel = 1.5, acc = 1.4)

        self.urx_rob2.robot.send_program("zero_ftsensor()")

        force_mod = [0,1,0,0,0,0]
        force_toq = [0,-8,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        time.sleep(2)

        while(True):
            try:
                force = self.urx_rob2.robot.get_tcp_force()
                print force[1]

                if abs(force[1]) > 8:
                    # print force[0], force[1]
                    self.urx_rob2.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.urx_rob2.robot.send_program("end_force_mode()")
                break


        cur = self.urx_rob2.robot.getl()
        cur[0] -= 0.05
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        force_mod = [0,1,0,0,0,0]
        force_toq = [0,-8,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        time.sleep(2)

        while(True):
            try:
                force = self.urx_rob2.robot.get_tcp_force()
                print force[1]

                if abs(force[1]) > 8:
                    # print force[0], force[1]
                    self.urx_rob2.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.urx_rob2.robot.send_program("end_force_mode()")
                break

        cur[0] += 0.08
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        force_mod = [0,1,0,0,0,0]
        force_toq = [0,-8,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        time.sleep(2)

        while(True):
            try:
                force = self.urx_rob2.robot.get_tcp_force()
                print force[1]

                if abs(force[1]) > 8:
                    # print force[0], force[1]
                    self.urx_rob2.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.urx_rob2.robot.send_program("end_force_mode()")
                break

        cur = self.urx_rob2.robot.getl()
        cur[1] += 0.18
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)


        self.urx_rob2.robot.movej(rob2_init_pose, vel = 1.5, acc = 1.4)#, vel = 1.5, acc = 1.4)


        self.urx_rob1.robot.set_digital_out(1, True)
        self.urx_rob1.robot.set_digital_out(0, False)
        time.sleep(1)

    def plate_motion(self):
        self.urx_rob2.robot.movej(sucktion_hold_rob2, vel=1.5, acc=1.4)

        cur = self.urx_rob2.robot.getl()
        cur[1] -= 0.05
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25, wait=False)

        self.urx_rob1.robot.movej(sucktion_tool_pre, vel=1.5, acc=1.4)

        cur = self.urx_rob1.robot.getl()
        cur[2] -= 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.gripper_move_and_wait(255)

        cur[2] += 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.robot.movej(sucktion_pose_1, vel=1.5, acc=1.4)
        self.urx_rob1.robot.movej(sucktion_pose_2, vel=1.5, acc=1.4)
        self.urx_rob1.robot.movej(sucktion_pose_3, vel=1.5, acc=1.4)

        #####

        self.urx_rob1.robot.set_digital_out(4, False)
        self.urx_rob1.robot.set_digital_out(5, False)
        self.urx_rob1.robot.set_digital_out(6, False)
        self.urx_rob1.robot.set_digital_out(7, False)

        self.urx_rob1.robot.send_program("zero_ftsensor()")

        self.urx_rob1.robot.set_digital_out(4, True)
        self.urx_rob1.robot.set_digital_out(5, True)

        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-10,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)

        

        while(True):
            try:
                force = self.urx_rob1.robot.get_tcp_force()

                if abs(force[2]) > 10:
                    print force[2]
                    self.urx_rob1.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.urx_rob1.robot.send_program("end_force_mode()")
                break

        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.robot.movej(sucktion_move_1, vel=1.05, acc=0.4)
        self.urx_rob1.robot.movej(sucktion_move_2, vel=1.05, acc=0.4)

        self.urx_rob1.suction_align()

        self.urx_rob1.robot.set_digital_out(4, False)
        self.urx_rob1.robot.set_digital_out(5, False)

        time.sleep(1)

        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.05
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.robot.movej(sucktion_pose_1, vel=1.05, acc=0.4)

        self.urx_rob1.robot.movej(sucktion_tool_pre, vel=1.05, acc=0.4)

        cur = self.urx_rob1.robot.getl()
        cur[2] -= 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.gripper_move_and_wait(0)

        cur[2] += 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        cur = self.urx_rob2.robot.getl()
        cur[1] += 0.05
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.robot.movej(rob1_init_pose, vel=1.5, acc=1.4)
        self.urx_rob2.robot.movej(rob2_init_pose, vel=1.5, acc=1.4)


    def fin_turn(self):
        self.urx_rob1.robot.set_digital_out(1, True)
        self.urx_rob1.robot.set_digital_out(0, True)
        time.sleep(1)

        self.urx_rob1.robot.movej(turn_point4_rob1, vel = 1.5, acc = 1.4, wait=False)

        while True:
            if self.urx_rob1.robot.get_digital_in(0):
                break

        self.urx_rob2.robot.movej(turn_point4_rob2, vel = 1.5, acc = 1.4)

        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.013
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25, wait=False)

        cur = self.urx_rob2.robot.getl()
        cur[2] += 0.013
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        cur = self.urx_rob1.robot.getl()
        cur[0] += 0.152
        cur[1] -= 0.1
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25, wait=False)

        cur = self.urx_rob2.robot.getl()
        cur[1] -= 0.15
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)


        

        self.urx_rob2._gripper_move_t(255)

        force_mod = [0,1,1,0,0,0]
        force_toq = [0,0,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        self.urx_rob1.gripper_move_and_wait(255)

        cur = self.urx_rob1.robot.getl()
        cur[1] -= 0.08
        self.urx_rob1.robot.movel(cur)

        self.urx_rob2.robot.send_program("end_force_mode()")

        self.urx_rob1.robot.set_digital_out(1, False)
        self.urx_rob1.robot.set_digital_out(0, False)
        time.sleep(1)

        while True:
            if self.urx_rob1.robot.get_digital_in(0):
                break

        self.urx_rob1.robot.set_digital_out(3, False)
        self.urx_rob1.robot.set_digital_out(2, False)
        time.sleep(1)

        self.urx_rob2.robot.send_program("zero_ftsensor()")

        force_mod = [1,0,1,0,1,0]
        force_toq = [0,0,10,0,2,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        curj = self.urx_rob1.robot.getj()
        curj[5] = turn_point5_rob1[5]

        self.urx_rob1.robot.movej(curj)

        self.urx_rob2.robot.send_program("end_force_mode()")

        cur1 = self.urx_rob1.robot.getl()
        cur2 = self.urx_rob2.robot.getl()

        cur2[5] = cur1[5]

        self.urx_rob2.robot.movel(cur2)

        force_mod = [1,0,0,0,0,0]
        force_toq = [0,0,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        cur = self.urx_rob1.robot.getl()
        cur[0] -= 0.25
        self.urx_rob1.robot.movel(cur)

        self.urx_rob1.robot.send_program("end_force_mode()")
        self.urx_rob2.robot.send_program('end_force_mode()')

        self.urx_rob1.robot.set_digital_out(1, True)
        self.urx_rob1.robot.set_digital_out(0, True)

        time.sleep(1)

        while True:
            if self.urx_rob1.robot.get_digital_in(0):
                break


        # force_mod = [0,0,1,0,1,0]
        # force_toq = [0,0,8,0,3.8,0] 

        # cmd_str  = "def go_down():"
        # cmd_str += "\tforce_mode_set_damping(0.00)\n"
        # cmd_str += "\twhile (True):\n"
        # cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        # cmd_str += "\t\tsync()\n"
        # cmd_str += "\tend\n"
        # cmd_str += "end\n"

        # self.urx_rob2.robot.send_program(cmd_str)

        force_mod = [0,0,1,0,1,0]
        force_toq = [0,0,8,0,-3.8,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        # cmd_str += "\t\tfreedrive_mode()\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)

        ############################################

        # force_mod = [0,0,0,0,1,0]
        # force_toq = [0,0,0,0,3.8,0]

        # cmd_str  = "def sprial():\n"
        # cmd_str += "\tforce_mode_set_damping(0.0)\n"
        # cmd_str += "\tthread Thread_1():\n"
        # cmd_str += "\tset_digital_out(6, False)\n"
        # cmd_str += "\t\twhile (True):\n"
        # cmd_str += "\t\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        # cmd_str += "\t\t\tsync()\n"
        # cmd_str += "\t\tend\n"
        # cmd_str += "\tend\n"
        # cmd_str += "\tglobal thrd = run Thread_1()\n"    
        # cmd_str = "\tcur = getl()\n"
        # cmd_str = "\tcur[2] = 0.4\n"
        # cmd_str = "\tmovel(cur)\n"
        # cmd_str += "\tend_force_mode()\n"
        # cmd_str += "\tset_digital_out(6, True)\n"
        # cmd_str += "\tkill thrd\n"
        # cmd_str += "end\n"

        # self.urx_rob2.robot.send_program(cmd_str)
        ###################################333

        

        cur = self.urx_rob2.robot.getl()
        cur[2] = 0.400
        self.urx_rob2.robot.movel(cur)

        while True:
            if self.urx_rob2.robot.getl()[2]<0.43:
                force_mod = [0,0,1,0,1,0]
                force_toq = [0,0,8,0,-3.8,0] 

                cmd_str  = "def go_down():"
                cmd_str += "\tforce_mode_set_damping(0.00)\n"
                cmd_str += "\twhile (True):\n"
                # cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
                cmd_str += "\t\tfreedrive_mode()\n"
                cmd_str += "\t\tsync()\n"
                cmd_str += "\tend\n"
                cmd_str += "end\n"

                self.urx_rob1.robot.send_program(cmd_str)
                break

        self.urx_rob1.gripper_move_and_wait(0)

        cur = self.urx_rob1.robot.getl()
        cur[1] += 0.100
        self.urx_rob1.robot.movel(cur, vel=1.05, acc=0.25)
        cur[2] += 0.100
        self.urx_rob1.robot.movel(cur, vel=1.05, acc=0.25)

        self.urx_rob2.gripper_move_and_wait(0)

        cur = self.urx_rob2.robot.getl()
        cur[2] -= 0.02
        self.urx_rob2.robot.movel(cur, vel=1.05, acc=0.25)
        cur[1] += 0.10
        self.urx_rob2.robot.movel(cur, vel=1.05, acc=0.25)

        self.urx_rob1.robot.movej(rob1_init_pose, vel=1.05, acc=1.4)
        self.urx_rob2.robot.movej(rob2_init_pose, vel=1.05, acc=1.4)


    def short_screw_pin(self, pin_pose):
        self.urx_rob1.robot.movej(pin_pose, vel = 1.5, acc = 1.4)

        self.urx_rob1.robot.set_digital_out(4, False)
        self.urx_rob1.robot.set_digital_out(5, False)
        self.urx_rob1.robot.set_digital_out(6, False)
        self.urx_rob1.robot.set_digital_out(7, False)

        time.sleep(0.1)

        self.urx_rob1.robot.set_digital_out(6, True)
        self.urx_rob1.robot.set_digital_out(4, True)



        self.urx_rob1.robot.send_program("zero_ftsensor()")

        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-1,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        # cmd_str += "\t\tfreedrive_mode()\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob1.robot.send_program(cmd_str)

        while(True):
            try:
                force = self.urx_rob1.robot.get_tcp_force()
                print force[0]

                if abs(force[2]) > 3:
                    # print force[0], force[1]
                    self.urx_rob1.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.urx_rob1.robot.send_program("end_force_mode()")
                break

        self.urx_rob1.robot.set_digital_out(6, False)
        self.urx_rob1.robot.set_digital_out(4, False)

        cur = self.urx_rob1.robot.getl()
        cur[2] += 0.2
        self.urx_rob1.robot.movel(cur, vel=1.05, acc=0.25)

        self.urx_rob1.robot.movej(screw_short_tool_pre1, vel = 1.5, acc = 1.4)

    def test_motion(self):

        # self.urx_rob1.robot.movej(rob1_init_pose, vel = 3, acc = 2)
        # self.urx_rob2.robot.movej(rob2_init_pose, vel = 1.5, acc = 1.4)

        # raw_input()

        # self.base_wood_pin_insert()

        # # print "wood fin"
        # # raw_input()

        # self.part2_insert_motion()
        
        # # print "part2 fin"
        # # raw_input()

        # self.part3_insert_motion()
        
        # # print "part3 fin"
        # # raw_input()

        # self.part4_insert_motion()

        # # print "part4 fin"
        # # raw_input()

        # self.part4_wood_pin_insert()

        # # print "part4-wood fin"
        # # raw_input()

        # self.part5_insert_motion()

        # # print "part5 fin"
        # # raw_input()

        # self.long_screw_insert1()

        # # print "insert1"
        # # raw_input()

        # self.long_screw_insert2()

        # # print "insert2"
        # # raw_input()

        # self.long_screw_insert3()

        # # print "insert3"
        # # raw_input()

        # self.table_screw_insert1()

        # # print "table_insert1"
        # # raw_input()

        # self.table_screw_insert2()

        # # print "table_insert2"
        # # raw_input()

        # self.table_screw_insert3()

        # # print "table_insert3"
        # # raw_input()

        #####

        self.urx_rob2.robot.movej(screw_hold_point1_rob2, vel = 1.5, acc = 1.4)

        self.urx_rob2.robot.send_program("zero_ftsensor()")

        force_mod = [0,1,0,0,0,0]
        force_toq = [0,-8,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)
        time.sleep(3)

        while(True):
            try:
                force = self.urx_rob2.robot.get_tcp_force()
                print force[1]

                if abs(force[1]) > 10:
                    # print force[0], force[1]
                    self.urx_rob2.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.urx_rob2.robot.send_program("end_force_mode()")
                break

        self.urx_rob2.robot.movej(screw_hold_point1_rob2, vel = 1.5, acc = 1.4)


        self.urx_rob2.robot.movej(screw_hold_point2_rob2, vel = 1.5, acc = 1.4)

        self.urx_rob2.robot.movej(screw_hold_point3_rob2, vel = 1.5, acc = 1.4)

        cur = self.urx_rob2.robot.getl()
        cur[1] -= 0.02
        self.urx_rob2.robot.movel(cur, vel=1.05, acc=0.25)



        self.urx_rob2.robot.send_program("zero_ftsensor()")

        force_mod = [1,0,0,0,0,0]
        force_toq = [-1,0,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        # cmd_str += "\t\tfreedrive_mode()\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        while(True):
            try:
                force = self.urx_rob2.robot.get_tcp_force()
                print force[0]

                if abs(force[0]) > 2:
                    # print force[0], force[1]
                    self.urx_rob2.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.urx_rob2.robot.send_program("end_force_mode()")
                break

        pose1 = self.urx_rob2.robot.getl()

        force_mod = [1,0,0,0,0,0]
        force_toq = [1,0,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        # cmd_str += "\t\tfreedrive_mode()\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        while(True):
            try:
                force = self.urx_rob2.robot.get_tcp_force()
                print force[0]

                if abs(force[0]) > 3:
                    # print force[0], force[1]
                    self.urx_rob2.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.urx_rob2.robot.send_program("end_force_mode()")
                break


        pose2 = self.urx_rob2.robot.getl()

        pose3 = pose2[0] - pose1[0]

        cur = self.urx_rob2.robot.getl()
        cur[0] -= (pose3/2)

        self.urx_rob2.robot.movel(cur, vel=1.05, acc=0.25)

        default = self.urx_rob2.robot.getl()

        cur = self.urx_rob2.robot.getl()
        cur[1] += 0.200
        self.urx_rob2.robot.movel(cur, vel=1.05, acc=0.25)

        self.urx_rob2.robot.movej(screw_hold_point4_rob2, vel = 1.5, acc = 1.4)

        cur = self.urx_rob2.robot.getl()
        cur[0] = default[0] - 0.085
        self.urx_rob2.robot.movel(cur, vel=1.05, acc=0.25)

        cur[1] -= 0.200
        self.urx_rob2.robot.movel(cur, vel=1.05, acc=0.25)

        cur[2] -= 0.100
        self.urx_rob2.robot.movel(cur, vel=1.05, acc=0.25)

        self.urx_rob2._gripper_move_t(255)

        force_mod = [0,1,0,0,0,0]
        force_toq = [0,0,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        # cmd_str += "\t\tfreedrive_mode()\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        time.sleep(1)

        force_mod = [0,1,1,0,0,0]
        force_toq = [0,-5,-30,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.00)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        # cmd_str += "\t\tfreedrive_mode()\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)



        self.urx_rob1.robot.movej(screw_short_tool_pre0, vel = 1.5, acc = 1.4)
        self.urx_rob1.robot.movej(screw_short_tool_pre1, vel = 1.5, acc = 1.4)
        self.urx_rob1.robot.movej(screw_short_tool_pre2, vel = 1.5, acc = 1.4)

        cur = self.urx_rob1.robot.getl()
        cur[2] -= 0.2
        self.urx_rob1.robot.movel(cur, vel=1.05, acc=0.25)

        cur[0] += 0.1
        self.urx_rob1.robot.movel(cur, vel=1.05, acc=0.25)

        self.urx_rob1.gripper_move_and_wait(255)

        cur[0] -= 0.1
        self.urx_rob1.robot.movel(cur, vel=1.05, acc=0.25)

        cur[2] += 0.2
        self.urx_rob1.robot.movel(cur, vel=1.05, acc=0.25)

        self.urx_rob1.robot.movej(screw_short_tool_pre1, vel = 1.5, acc = 1.4)

        ####나사 가져오기#####

        self.short_screw_pin(screw_pin_pose1)

        ########################

        self.urx_rob1.robot.movej(part4_hole_pre, vel = 1.5, acc = 1.4)

        cur = self.urx_rob1.robot.getl()
        cur[0] = -1*default[0]
        self.urx_rob1.robot.movel(cur, vel=1.05, acc=0.25)

        cur[1] -= 0.250
        self.urx_rob1.robot.movel(cur, vel=1.05, acc=0.25)

        cur = part4_hole_1
        cur[0] = (-1 * default[0]) - 0.01628
        cur[1] -= 0.01 
        self.urx_rob1.robot.movel(cur, vel=0.5, acc=0.25)

        print "hole 1"
        raw_input()


        cur = part4_hole_2
        cur[0] = (-1 * default[0]) +0.0019
        cur[1] -= 0.01 
        self.urx_rob1.robot.movel(cur)

        print "hole 2"
        raw_input()

        cur = part4_hole_3
        cur[0] = (-1 * default[0]) - 0.01916
        cur[1] += 0.01 
        self.urx_rob1.robot.movel(cur)

        print "hole 3"
        raw_input()

        cur = part4_hole_4
        cur[0] = (-1 * default[0]) - 0.08919
        cur[1] += 0.01 
        self.urx_rob1.robot.movel(cur)

        print self.urx_rob1.robot.getl()
        
        cur = part4_hole_1
        self.urx_rob1.robot.movel(cur)

        print "end"


        

        


def main():
    mt = Motion_test()
    print "go?"
    raw_input()
    mt.test_motion()



  

if __name__ == '__main__':
    
    main()