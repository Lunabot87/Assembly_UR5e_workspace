#-*- coding:utf-8 -*-
import time
import urx
import math as m
import numpy as np
import copy
import random
import time

class UrxMotion():

    def __init__(self, robot_ip):
        self.reset()
        self.rob1.send_program(self._set_gripper())
        self.rob2.send_program(self._set_gripper())
        time.sleep(3)

        self.pin_pose = [[-0.03045, -1.96946, -1.80458 -0.93844 1.5692 -0.02784],[-0.08934, -1.99052, -1.77228 -0.94964 1.5691 -0.08679],[-0.03276, -1.89556, -1.91412 -0.90275 1.5692 -0.02986],[-0.09810, -1.91374, -1.88767 -0.91104 1.5692 -0.09525],[-0.03553, -1.81349, -2.02856 -0.87075 1.5694 -0.03246],[-0.10267, -1.83680, -1.99677 -0.87891 1.5693 -0.09958]]
        self.pin_pre_pose = [[-0.02975, -1.784503, -1.282465, -1.6455, 1.5691, -0.027005],[-0.08853, -1.809195, -1.282967, -1.6204, 1.5691, -0.086079],[-0.03197, -1.695997, -1.348806, -1.6677, 1.5692, -0.029000],[-0.09740, -1.714498, -1.384382, -1.6136, 1.5692, -0.094405],[-0.03477, -1.602397, -1.402856, -1.7075, 1.5694, -0.031513],[-0.10202, -1.621245, -1.527932, -1.5632, 1.5694, -0.098787]]

        self.hole_pose = [[-1.298815552, -1.901123662, -1.85568714, -0.955488995, 1.5698800087, 0.2749400138],[-1.417055908, -1.899667402, -1.92237186, -0.890888051, 1.5710926055, 0.1558179855],[-0.642283741, -2.048429628, -1.69451618, -0.970111684, 1.5706362724, 0.9299578666]]

        self.pre_hand_over = [[-1.2928, -1.4172, -1.4987, -0.2354, 1.2902, 1.5710] ,[-0.7087, -1.0365, -1.7401, -0.3599, 0.7054, 1.5706]]
        self.hand_over = [[-1.3860, -1.8260, -1.0690, -0.2458, 1.3844, 1.5692],[-1.1955, -1.6401, -1.2864, -0.2102, 1.1927, 1.5721]]


        

    def reset(self):
        connected = False 
        while not connected:
            try:         
                self.rob1 = urx.URRobot("192.168.13.101", use_rt=True) 
                self.rob2 = urx.URRobot("192.168.13.100", use_rt=True)        
                connected = True    
            except:     
                time.sleep(1)


    def force_control(self, robot):
        robot.send_program("zero_ftsensor()")
        time.sleep(0.2)

        print "="*20 +"go_down"+"="*20
      
        force_mod = [1,1,1,0,0,0]
        force_toq = [0,0,-1,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        robot.send_program(cmd_str)
        time.sleep(0.2)

        start = robot.getl()
        while(True):
            try:
                force = robot.get_tcp_force()

                if start[2] - robot.getl()[2] > 0.04:
                    robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                robot.send_program("end_force_mode()")
                break


        time.sleep(0.1)


    def _set_gripper(self, speed = 255, force = 100):

        msg  = "def UR5_gripper_set():\n"
        msg += "\tsocket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
        #set input and output voltage ranges
        msg += "\tset_analog_inputrange(0,0)\n"
        msg += "\tset_analog_inputrange(1,0)\n"
        msg += "\tset_analog_inputrange(2,0)\n"
        msg += "\tset_analog_inputrange(3,0)\n"
        msg += "\tset_analog_outputdomain(0,0)\n"
        msg += "\tset_analog_outputdomain(1,0)\n"
        msg += "\tset_tool_voltage(0)\n"
        msg += "\tset_runstate_outputs([])\n"
        #set payload, speed and force
        msg += "\tset_payload(1.1)\n"
        msg += "\tsocket_set_var(\"SPE\",%d,\"gripper_socket\")\n"%(speed)
        msg += "\tsync()\n"
        msg += "\tsocket_set_var(\"FOR\",%d,\"gripper_socket\")\n"%(force)
        msg += "\tsync()\n"
        #initialize the gripper
        msg += "\tsocket_set_var(\"ACT\", 1, \"gripper_socket\")\n"
        msg += "\tsync()\n"
        msg += "\tsocket_set_var(\"GTO\", 1, \"gripper_socket\")\n"
        msg += "\tsync()\n"
        msg += "\tsocket_set_var(\"POS\", 0, \"gripper_socket\")\n"
        msg += "\tsync()\n"
        msg += "\tsleep(0.5)\n"
        msg += "\ttextmsg(\"gripper setting complete\")\n"
        msg += "end\n"

        print("gripper_set")

        return msg

    def _gripper_move(self, pos):
        msg  = "def UR5_gripper_action():\n"
        msg += "\tsocket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
        msg += "\tsocket_set_var(\"POS\", %s, \"gripper_socket\")\n"%(str(pos)) #catched chair!
        msg += "\trq_pos_1 = socket_get_var(\"POS\",\"gripper_socket\")\n"
        msg += "\twhile True:\n"
        msg += "\t\tsleep(0.01)\n"
        msg += "\t\trq_pos = socket_get_var(\"POS\",\"gripper_socket\")\n"
        #msg += "\t\ttextmsg(\"rq_pos:\", rq_pos)\n"
        msg += "\t\tif norm(rq_pos_1 - rq_pos) < 0.5:\n"
        msg += "\t\t\tset_digital_out(10, True)\n"
        msg += "\t\t\tbreak\n"
        msg += "\t\telse:\n"
        msg += "\t\t\trq_pos_1 = rq_pos\n"
        msg += "\t\tend\n"
        msg += "\tend\n"
        msg += "end\n"

        return msg

    def gripper_move_and_wait(self, robot ,pos):
        robot = robot

        robot.send_program(self._gripper_move(pos))

        while True:
            if robot.get_digital_out(10) != 0:
                robot.send_program("set_digital_out(7, False)")
                print "gripper_move_and_wait complete"
                break
            else:
                continue


    def init_pose(self, robot):
        robot.movej([-1.5708006064044397, -1.04721291482959, -1.5707988739013672, -1.5739270649352015, 1.5707921981811523, 5.3882598876953125e-05])


    def pose_z_axis_up(self, robot, z_offset):
        goal_pose = robot.getl()
        goal_pose[2] += z_offset

        robot.movel(goal_pose)

    def pose_y_axis_up(self, robot, z_offset):
        goal_pose = robot.getl()
        goal_pose[1] += z_offset

        robot.movel(goal_pose)


    def handover(self, hole_num):

        self.rob1.movej(self.hand_over_pre_pose[0])

        self.rob2.movej(self.hand_over_pre_pose[1])

        self.rob1.movej(self.hand_over_pose[0])

        self.rob2.movej(self.hand_over_pose[1])

        if hole_num == 1:
            self.gripper_move_and_wait(self.rob1, 255)

            self.gripper_move_and_wait(self.rob2, 0)

            self.rob1.movej(self.hand_over_pre_pose[0])

            self.rob2.movej(self.hand_over_pre_pose[1])

            self.init_pose(self.rob2)

        else:
            self.gripper_move_and_wait(self.rob2, 255)

            self.gripper_move_and_wait(self.rob1, 0)

            self.rob1.movej(self.hand_over_pre_pose[0])

            self.rob2.movej(self.hand_over_pre_pose[1])

            self.init_pose(self.rob1)



    def insert_hole(self, hole_num, pin_num):
        if hole_num == 1:
            #tool table movement
            time.sleep(10)

            self.gripper_move_and_wait(self.rob1, 0) #gripper open
            self.gripper_move_and_wait(self.rob2, 0) #gripper open

            self.rob2.movej(self.pin_pre_pose[pin_num - 1])

            self.gripper_move_and_wait(self.rob2, 100)

            self.rob2.movej(self.pin_pose[pin_num - 1])

            self.gripper_move_and_wait(self.rob2, 255)

            self.pose_z_axis_up(self.rob2, 0.3)

            self.handover(hole_num)

            self.rob1.movej(self.hole_pose[hole_num - 1])

            self.force_mod(rob1)

            self.gripper_move_and_wait(self.rob1, 0)

            self.pose_z_axis_up(self.rob1, 0.3)

            self.init_pose(self.rob1)


        else:
            #tool table movement
            time.sleep(10)

            self.gripper_move_and_wait(self.rob1, 0) #gripper open
            self.gripper_move_and_wait(self.rob2, 0) #gripper open

            self.rob1.movej(self.pin_pre_pose[pin_num - 1])

            self.gripper_move_and_wait(self.rob2, 100)

            self.rob1.movej(self.pin_pose[pin_num - 1])

            self.gripper_move_and_wait(self.rob1, 255)

            self.pose_z_axis_up(self.rob1, 0.3)

            self.handover(hole_num)

            self.rob2.movej(self.hole_pose[hole_num - 1])

            self.force_mod(self.rob2)

            self.gripper_move_and_wait(self.rob2, 0)

            self.pose_z_axis_up(self.rob2, 0.3)

            self.init_pose(self.rob2)

    def main(): 
        hole_num = []#data input

        pin_num = []

        for i in range(3):
            self.insert_hole(hole_num[i], pin_num[i])

        print "insert_end"

    


    

if __name__ == '__main__':
    robot = UrxMotion()

    robot.main()

