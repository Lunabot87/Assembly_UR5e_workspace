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

    def test_motion(self):
        # self.urx_rob2.robot.movej(part2_hold_pre_2)
        # cur = self.urx_rob2.robot.getl()
        # cur[2] -= 0.2
        # self.urx_rob2.robot.movel(cur)

        ###### part2 insert ######
        # self.urx_rob1.robot.movej(part2_grap_pre, vel = 1.05, acc = 1.4)
        # cur = self.urx_rob1.robot.getl()
        # cur[2] -= 0.2
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 1.4)
        # self.urx_rob1.gripper_move_and_wait(255)
        # cur = self.urx_rob1.robot.getl()
        # cur[2] += 0.2
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 1.4)
        # self.urx_rob1.robot.movej(part2_rotate, vel = 1.05, acc = 1.4)
        # self.urx_rob1.robot.movej(part2_insert, vel = 1.05, acc = 1.4)
        # self.urx_rob1.spiral_motion()


        #####gripper open#####
        # self.urx_rob1.gripper_move_and_wait(0)

        ##init pose##

        # cur = self.urx_rob1.robot.getl()
        # cur[2] += 0.4
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 1.4)
        # self.mg_rob1.get_named_target_values("rob1_init_pose")

        #############################


        ###### part3 insert ######

        # self.urx_rob2.robot.movej(part3_grap_pre, vel = 1.05, acc = 1.4)

        # cur = self.urx_rob2.robot.getl()
        # cur[2] -= 0.2
        # self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 1.4)
        # self.urx_rob2.gripper_move_and_wait(255)
        # cur = self.urx_rob2.robot.getl()
        # cur[2] += 0.2
        # self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part3_rotate, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part3_insert, vel = 1.05, acc = 1.4)
        # self.urx_rob2.spiral_motion()


        #####gripper open#####
        # self.urx_rob2.gripper_move_and_wait(0)

        ############################

        ### part3  wood pin insert ###

        # self.urx_rob1.robot.movej(rob1_101350_pre_2, vel = 1.05, acc = 1.4)
        # print "grip?"
        # raw_input()
        # self.urx_rob1.robot.movej(part3_hole3, vel = 1.05, acc = 1.4)
        # self.urx_rob1.spiral_motion()
        # self.urx_rob1.gripper_move_and_wait(180)
        # cur = self.urx_rob1.robot.getl()
        # cur[2] += 0.1
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 1.4)
        # self.urx_rob1.robot.movej(rob1_101350_pre_2, vel = 1.05, acc = 1.4)
        # print "grip?"
        # raw_input()
        # self.urx_rob1.robot.movej(part3_hole4, vel = 1.05, acc = 1.4)
        # self.urx_rob1.spiral_motion()
        # self.urx_rob1.gripper_move_and_wait(180)
        # cur = self.urx_rob1.robot.getl()
        # cur[2] += 0.1
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 1.4)

        ##################################

        ##### part4 move #####

        # self.urx_rob1.robot.movej(part4_move_grap, vel = 1.05, acc = 1.4)
        # self.urx_rob1.gripper_move_and_wait(120)
        # cur = self.urx_rob1.robot.getl()
        # cur[2] -= 0.2
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)
        # self.urx_rob1.gripper_move_and_wait(255)
        # cur = self.urx_rob1.robot.getl()
        # cur[2] += 0.2
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)
        # self.urx_rob1.robot.movej(part4_move_point, vel = 1.05, acc = 1.4)


        # ###### part4 insert ######

        # self.urx_rob2.robot.movej(part4_grap_pre, vel = 1.05, acc = 1.4)

        # cur = self.urx_rob2.robot.getl()
        # cur[1] -= 0.2
        # self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)
        # self.urx_rob2.gripper_move_and_wait(255)

        # self.urx_rob1.gripper_move_and_wait(120)
        # cur = self.urx_rob1.robot.getl()
        # cur[2] -= 0.1
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        # cur = self.urx_rob2.robot.getl()
        # cur[1] += 0.2
        # self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)


        # self.urx_rob2.robot.movej(part4_rotate, vel = 1.05, acc = 1.4)
        # self.urx_rob2.robot.movej(part4_insert, vel = 1.05, acc = 1.4)
        # self.urx_rob2.spiral_motion()

        # self.urx_rob2.gripper_move_and_wait(0)
        # cur = self.urx_rob2.robot.getl()
        # cur[2] += 0.2
        # self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        # ###########################

        # ###### part4 wood pin hold ######
        # self.urx_rob1.robot.movej(part4_hold_pre, vel = 1.05, acc = 1.4)
        # self.urx_rob1.gripper_move_and_wait(120)
        # cur = self.urx_rob1.robot.getl()
        # cur[1] -= 0.2
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)
        # self.urx_rob1.gripper_move_and_wait(255)

        ##################################


        ###### part6 start init #####
        # self.urx_rob1.gripper_move_and_wait(120)
        # cur = self.urx_rob1.robot.getl()
        # cur[1] += 0.3
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)
        # self.urx_rob1.gripper_move_and_wait(0)
        # cur[2] += 0.3
        # self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob1.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(rob2_init_pose, vel = 1.05, acc = 1.4)

        ###############################

        ###### part6 insert ##########
        self.urx_rob2.robot.movej(part5_grap_pre, vel = 1.05, acc = 1.4)
        cur = self.urx_rob2.robot.getl()
        cur[2] -= 0.4
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)
        self.urx_rob2.gripper_move_and_wait(255)
        cur[2] += 0.4
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)
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

        print "down?"
        raw_input()

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

        force_mod = [1,0,1,1,1,0]
        force_toq = [0,0,-15,0,2,0] 

        print "down?"
        raw_input()

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.05)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)

        #######part6_part2_insert motion#########
        self.urx_rob1.robot.movej(rob1_init_pose, vel = 1.05, acc = 1.4)

        self.urx_rob1.robot.movej(part6_pose1_1, vel = 1.05, acc = 1.4)
        self.urx_rob1.robot.movej(part6_pose1_2, vel = 1.05, acc = 1.4)

        cur = self.urx_rob1.robot.getl()
        cur[0] -= 0.2
        self.urx_rob1.robot.movel(cur, vel = 1.05, acc = 0.25)


        self.urx_rob1.gripper_move_and_wait(255)

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

        force_mod = [1,1,1,0,1,0]
        force_toq = [0,0,-15,0,-2,0] 

        print "down?"
        raw_input()

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

        force_mod = [1,1,1,0,0,0]
        force_toq = [0,0,-15,0,0,0] 

        print "down?"
        raw_input()

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.05)\n"
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

        self.urx_rob2.robot.movej(part6_pose3_1, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(part6_pose3_2, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(part6_pose3_3, vel = 1.05, acc = 1.4)

        cur = self.urx_rob2.robot.getl()
        cur[0] += 0.1
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        self.urx_rob2.gripper_move_and_wait(255)

        self.urx_rob2.spiral_motion()

        self.urx_rob2.gripper_move_and_wait(0)

        self.urx_rob2.robot.movej(part6_pose3_3, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(part6_pose3_2, vel = 1.05, acc = 1.4)
        self.urx_rob2.robot.movej(part6_pose3_1, vel = 1.05, acc = 1.4)


        self.urx_rob2.robot.movej(rob2_init_pose, vel = 1.05, acc = 1.4)


        self.urx_rob2.robot.movej(part6_pose2_1, vel = 1.05, acc = 1.4)
        cur = self.urx_rob2.robot.getl()
        cur[2] -= 0.1
        self.urx_rob2.robot.movel(cur, vel = 1.05, acc = 0.25)

        print "down?"
        raw_input()

        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-50,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.05)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"

        self.urx_rob2.robot.send_program(cmd_str)



def main():
    mt = Motion_test()
    print "go?"
    raw_input()
    mt.test_motion()



  

if __name__ == '__main__':
    
    main()