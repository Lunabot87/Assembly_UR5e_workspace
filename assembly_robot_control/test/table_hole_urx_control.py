#-*- coding:utf-8 -*-
import time
import urx
import math as m
import numpy as np
import copy
import random

class UrxMotion():

    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.reset()
        self.robot.send_program(self._set_gripper())
        time.sleep(3)


    def reset(self):
        connected = False 
        while not connected:
            try:         
                self.robot = urx.URRobot(self.robot_ip, use_rt=True)        
                connected = True    
            except:     
                time.sleep(1)  


    def force_control(self):
        self.robot.send_program("zero_ftsensor()")
        time.sleep(0.2)

        print "="*20 +"go_down"+"="*20
      
        force_mod = [0,0,1,0,pitch,0]
        force_toq = [0,0,-5,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        self.robot.send_program(cmd_str)
        time.sleep(0.2)


        while(True):
            try:
                force = self.robot.get_tcp_force()

                if abs(force[2]) > 3:
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
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
        msg += "\t\t\tset_digital_out(7, True)\n"
        msg += "\t\t\tbreak\n"
        msg += "\t\telse:\n"
        msg += "\t\t\trq_pos_1 = rq_pos\n"
        msg += "\t\tend\n"
        msg += "\tend\n"
        msg += "end\n"

        return msg


    def init_pose(self):
        self.robot.movej([-1.5708006064044397, -1.04721291482959, -1.5707988739013672, -1.5739270649352015, 1.5707921981811523, 5.3882598876953125e-05])


    def pose_z_axis_up(self, z_offset):
        goal_pose = self.robot.getl()
        goal_pose[2] += z_offset

        self.robot.movel(goal_pose)

    def pose_y_axis_up(self, z_offset):
        goal_pose = self.robot.getl()
        goal_pose[1] += z_offset

        self.robot.movel(goal_pose)



def main(): 
    rob1 = UrxMotion("192.168.13.101")
    rob2 = UrxMotion("192.168.13.100")

    rob1.init_pose()
    rob2.init_pose()

    rob1._gripper_move(0) #gripper open
    rob2._gripper_move(0) #gripper open

    rob1.movej(-0.12076789537538701, -1.6519357166686, -1.581700325012207, -1.4943846029094239, 1.5664734840393066, -0.10190803209413701)

    #pin1 pre pose
    rob1.movej()

    rob1._gripper_move(100) #gripper open

    rob1.pose_z_axis_up(0.3)


    #hand_over_pre_pose
    rob1.movej()
    rob2.movej()


    rob1.pose_y_axis_up(-0.3)
    rob2.pose_y_axis_up(-0.3)


    rob2._gripper_move(255)
    rob1._gripper_move(0)

    rob1.pose_y_axis_up(0.3)
    rob2.pose_y_axis_up(0.3)

    rob1.init_pose()

    #hole1 joint
    rob2.movej()   

    rob2.force_control()

    rob2.pose_z_axis_up(0.05)

    rob2._gripper_move(0) #gripper open

    rob2.init_pose()


    rob1._gripper_move(0) #gripper open
    rob2._gripper_move(0) #gripper open

    rob1.movej(-0.12076789537538701, -1.6519357166686, -1.581700325012207, -1.4943846029094239, 1.5664734840393066, -0.10190803209413701)

    #pin1 pre pose
    rob1.movej()

    rob1._gripper_move(100) #gripper open

    rob1.pose_z_axis_up(0.3)


    #hand_over_pre_pose
    rob1.movej()
    rob2.movej()


    rob1.pose_y_axis_up(-0.3)
    rob2.pose_y_axis_up(-0.3)


    rob2._gripper_move(0)
    rob1._gripper_move(255)

    rob1.pose_y_axis_up(0.3)
    rob2.pose_y_axis_up(0.3)

    rob1.init_pose()

    #hole1 joint
    rob2.movej()   

    rob2.force_control()

    rob2.pose_z_axis_up(0.05)

    rob2._gripper_move(255) #gripper open


    

if __name__ == '__main__':
    main()

