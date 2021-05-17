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
        # self.robot.send_program(self._set_gripper())
        time.sleep(3)

    def reset(self):
        connected = False 
        while not connected:      
            try:         
                self.robot = urx.URRobot(self.robot_ip, use_rt=True)        
                connected = True    
            except:     
                time.sleep(0.5)  

    def _format_move(self, command, tpose, acc, vel, radius=0, prefix=""):
        tpose = [round(i, 6) for i in tpose]
        tpose.append(acc)
        tpose.append(vel)
        tpose.append(radius)
        return "{}({}[{},{},{},{},{},{}], a={}, v={}, r={})".format(command, prefix, *tpose)  

    def _get_spiral_cmd(self, initial, reverse = False, screw = False):
        vel = 0.5
        acc = 0.25
        radius = 0.0005

        if screw:
            f = 30
        else:
            f = 12

        if reverse:
            axis = 1
        else:
            axis = 0

        force_mod = [0,0,1,axis,0,axis]
        force_toq = [0,0,-1*f,0,0,1] if reverse is False else [0,0,f,0,0,1]


        # spiral motion
        R = 0.008  #0.006 #0.003
        revolution = 10 #8 #4
        dth = m.pi/10
        th_len_1 = int(revolution*2*m.pi/dth) # len(th_array)-1
        dr = R/th_len_1 # R*0.005

        th_array = np.arange(0, revolution*2*m.pi+dth, step=dth)    
        r_array = np.arange(0, R+dr, step=dr)    
        th_array = np.concatenate((th_array, np.arange(dth, 2*m.pi+dth, step=dth)), axis=None)
        r_array = np.concatenate((r_array, np.ones(th_len_1/revolution)*R), axis=None)

        # print th_array

        spiral_list = []
        spiral_list.append(initial)
        i = 0
        for th in th_array:
            current = copy.deepcopy(initial)
            # current[0] += R*m.cos(th) - R 
            # current[1] += R*m.sin(th)
            current[0] += r_array[i]*m.cos(th) 
            current[1] += r_array[i]*m.sin(th)
            spiral_list.append(current)
            i += 1

        # cmd
        cmd_str  = "def sprial():\n"
        cmd_str += "\tforce_mode_set_damping(0.0)\n"
        cmd_str += "\tthread Thread_1():\n"
        cmd_str += "\tset_digital_out(6, False)\n"
        cmd_str += "\t\twhile (True):\n"
        cmd_str += "\t\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\t\tsync()\n"
        cmd_str += "\t\tend\n"
        cmd_str += "\tend\n"
        cmd_str += "\tglobal thrd = run Thread_1()\n"    

        for idx, pose in enumerate(spiral_list):
            if idx == (len(spiral_list) - 1):
                radius = 0
            test_cmd = self._format_move("movej", pose, acc, vel, radius, prefix="p") + "\n"
            cmd_str += test_cmd
        
        cmd_str += "\tend_force_mode()\n"
        cmd_str += "\tset_digital_out(6, True)\n"
        cmd_str += "\tkill thrd\n"
        cmd_str += "end\n"

        return cmd_str


    # def wait_for_move()

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


    def _gripper_move(self, pos, force = False):
        msg  = "def UR5_gripper_action():\n"

        if force:
            msg += "\tthread Thread_1():\n"
            msg += "\t\twhile (True):\n"
            msg += "\t\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+ '[0,1,0,0,0,0]' +"," + '[0,0,0,0,0,0]' +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
            msg += "\t\t\tsync()\n"
            msg += "\t\tend\n"
            msg += "\tend\n"
            msg += "\tglobal thrd = run Thread_1()\n"


        msg += "\tsocket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
        msg += "\tsocket_set_var(\"POS\", %s, \"gripper_socket\")\n"%(str(pos)) #catched chair!
        msg += "\trq_pos_1 = socket_get_var(\"POS\",\"gripper_socket\")\n"
        msg += "\twhile True:\n"
        msg += "\t\tsleep(0.01)\n"
        msg += "\t\trq_pos = socket_get_var(\"POS\",\"gripper_socket\")\n"
        #msg += "\t\ttextmsg(\"rq_pos:\", rq_pos)\n"
        msg += "\t\tif norm(rq_pos_1 - rq_pos) < 0.5:\n"
        msg += "\t\t\tset_digital_out(6, True)\n"
        msg += "\t\t\tbreak\n"
        msg += "\t\telse:\n"
        msg += "\t\t\trq_pos_1 = rq_pos\n"
        msg += "\t\tend\n"
        msg += "\tend\n"
        msg += "end\n"

        return msg


    def _gripper_move_t(self, pos, force = False):
        msg  = "def UR5_gripper_action():\n"

        if force:
            msg += "\tthread Thread_1():\n"
            msg += "\t\twhile (True):\n"
            msg += "\t\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+ '[0,1,0,0,0,0]' +"," + '[0,0,0,0,0,0]' +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
            msg += "\t\t\tsync()\n"
            msg += "\t\tend\n"
            msg += "\tend\n"
            msg += "\tglobal thrd = run Thread_1()\n"


        msg += "\tsocket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
        msg += "\tsocket_set_var(\"POS\", %s, \"gripper_socket\")\n"%(str(pos)) #catched chair!
        msg += "\trq_pos_1 = socket_get_var(\"POS\",\"gripper_socket\")\n"
        msg += "\tsleep(0.01)\n"
        msg += "\trq_pos = socket_get_var(\"POS\",\"gripper_socket\")\n"
        #msg += "\t\ttextmsg(\"rq_pos:\", rq_pos)\n"
        msg += "end\n"

        self.robot.send_program(msg)
        time.sleep(0.1)

    def suction_attach(self, force_mod, force_toq):

        self.robot.send_program("zero_ftsensor()")
        time.sleep(0.2)


        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        # cmd_str += "\tforce_mode_set_damping(0)\n"
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
                # print force[2]
                if force[2] > 5:
                    # print force[2]
                    time.sleep(1)
                    self.robot.send_program("end_force_mode()")
                    return True
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break



    def torque_mode(self, force_mod, force_toq, tool = False):

        # self.robot.send_program("zero_ftsensor()")
        # time.sleep(0.2)

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        # cmd_str += "\tforce_mode_set_damping(0)\n"
        cmd_str += "\twhile (True):\n"
        if tool is not True:
            cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        else:
            cmd_str += "\t\tforce_mode(tool_pose(), "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        self.robot.send_program(cmd_str)
        time.sleep(0.2)


    def freedrive_mode(self):

        cmd_str  = "def freedrive():"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tfreedrive_mode()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        self.robot.send_program(cmd_str)
        time.sleep(0.2)


    def gripper_move_and_wait(self ,pos, force = False):
        robot = self.robot

        robot.send_program(self._gripper_move(pos, force = force))

        while True:
            if self.robot.get_digital_out(6) != 0:
                self.robot.send_program("set_digital_out(6, False)")
                print "gripper_move_and_wait complete"
                break
            else:
                continue


    def spiral_motion(self, pitch = 0):

        #############################error test################################
        # move to initial pose
        # error = 0.006
        # random_error_x = error*random.uniform(-1, 1)
        # random_error_y = error*random.uniform(-1, 1)
        # start_pose[0] += -error #random_error_x
        # start_pose[1] += 0 #random_error_y
        # print "error_x={}, error_y={}".format(random_error_x, random_error_y)
        #######################################################################

        # robot = self.robot

        # robot.movel(start_pose, wait=True)

        # while(True):
        #     current = robot.getl()
        #     dist = np.linalg.norm(np.array(current)-np.array(start_pose))
        #     if dist < 0.001:
        #         break
        self.reset()
        time.sleep(1)

        self.robot.send_program("zero_ftsensor()")
        time.sleep(0.2)
        # print self.robot.get_tcp_force()


        # go down
        print "="*20 +"go_down"+"="*20
      

        force_mod = [0,0,1,0,pitch,0]
        force_toq = [0,0,-5,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        # cmd_str += "\tforce_mode_set_damping(0)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        self.robot.send_program(cmd_str)
        time.sleep(0.2)

        # print self.robot.get_tcp_force()

        while(True):
            try:
                # print "try"
                force = self.robot.get_tcp_force()
                # print force[2]
                # print force[2]
                if abs(force[2]) > 3:
                    # print force[2]
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break


        time.sleep(0.1)

        # spiral    
        print "="*20 +"spiral"+"="*20
        initial = self.robot.getl()
        spiral_cmd = self._get_spiral_cmd(initial)
        self.robot.send_program(spiral_cmd)
        time.sleep(0.1)

        while(True):
            try:
                force = self.robot.get_tcp_force()
                # print force[0], force[1]
                digi = self.robot.get_digital_out(6)
                # print "digi : {0}".format(type(digi))

                if digi > 0: #spiral 실패시
                    self.robot.send_program("set_digital_out(6, False)")
                    break 

                if abs(force[0]) > 25 or abs(force[1]) > 25:
                    # print force[0], force[1]
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break
        time.sleep(0.1)

        # real insert
        print "="*20 +"real_insert"+"="*20
        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-50,0,0,0]
        cmd_str  = "def real_insert():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.2, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        self.robot.send_program(cmd_str)


        ##############################수정 필요할수 있음###########################

        while(True):
            try:
                force = self.robot.get_tcp_force()
                # print force[2]
                if force[2] > 50:
                    # print force[2]
                    time.sleep(1)
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break

        ########################################################################

        # self.gripper_move_and_wait(0)

        # post_pose = self.robot.getl()
        # post_pose[2] += 0.3
        # self.robot.movel(post_pose)

        # while(True):
        #     current = self.robot.getl()
        #     dist = np.linalg.norm(np.array(current)-np.array(post_pose))
        #     if dist < 0.001:
        #         break
        return True

    def screw_motion(self, pitch = 0):

        # print "reset"
        # raw_input()


        self.reset()
        time.sleep(1)

        # print "zero_ftsensor"
        # raw_input()


        self.robot.send_program("zero_ftsensor()")
        time.sleep(0.2)
        # print self.robot.get_tcp_force()

        # print "go_down"
        # raw_input()

        # go down


        print "="*20 +"go_down"+"="*20
        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-5,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        # cmd_str += "\tforce_mode_set_damping(0)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0,0,0,0,0,0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        self.robot.send_program(cmd_str)
        time.sleep(0.5)

        # print self.robot.get_tcp_force()

        while(True):
            try:
                # print "try"
                force = self.robot.get_tcp_force()
                # print "force z : {0}".format(force[2])
                # print force[2]
                if abs(force[2]) > 8:
                    # print force[2]
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break


        # spiral    
        print "="*20 +"spiral"+"="*20
        initial = self.robot.getl()
        spiral_cmd = self._get_spiral_cmd(initial, screw = True)
        self.robot.send_program(spiral_cmd)

        while(True):
            try:
                force = self.robot.get_tcp_force()
                # print force[0], force[1]
                digi = self.robot.get_digital_out(6)
                # print "digi : {0}".format(type(digi))
                if digi > 0: #spiral 실패시
                    self.robot.send_program("set_digital_out(6, False)")
                    return False

                if abs(force[0]) > 40 or abs(force[1]) > 40:
                    # print force[0], force[1]
                    self.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break
        time.sleep(0.1)

        # print "real_insert now?"
        # raw_input()

        

    def screw_motion_insert(self):

        print "="*20 +"real_insert"+"="*20
        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-50,0,0,0]
        cmd_str  = "def real_insert():"
        cmd_str += "\tforce_mode_set_damping(0.000)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.2, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        self.robot.send_program(cmd_str)

        # print "reverse now?"
        # raw_input()

    def screw_motion_reverse(self):

        print "="*20 +"spiral_reverse"+"="*20
        initial = self.robot.getl()
        init_pose = copy.deepcopy(initial)
        spiral_cmd = self._get_spiral_cmd(init_pose, reverse=True)
        self.robot.send_program(spiral_cmd)

        while(True):
            if (abs(self.robot.getl()[2]) - abs(init_pose[2])) > 0.1:
                self.robot.send_program("end_force_mode()")
                break


    def suction_align(self):
        self.reset()
        time.sleep(1)

        self.robot.send_program("zero_ftsensor()")
        time.sleep(0.2)

        print "="*20 +"go_down"+"="*20
        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-5,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        # cmd_str += "\tforce_mode_set_damping(0)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0,0,0,0,0,0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        self.robot.send_program(cmd_str)
        time.sleep(0.5)

        # print self.robot.get_tcp_force()

        while(True):
            try:
                # print "try"
                force = self.robot.get_tcp_force()
                # print "force z : {0}".format(force[2])
                # print force[2]
                if abs(force[2]) > 8:
                    # print force[2]
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break


        print "="*20 +"go_front"+"="*20
        force_mod = [0,1,0,0,0,1]
        force_toq = [0,-5,0,0,0,-0.1] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        # cmd_str += "\tforce_mode_set_damping(0)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0,0,0,0,0,0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        self.robot.send_program(cmd_str)
        time.sleep(0.5)

        # print self.robot.get_tcp_force()

        while(True):
            try:
                # print "try"
                force = self.robot.get_tcp_force()
                # print "force z : {0}".format(force[2])
                # print force[2]
                if abs(force[1]) > 8:
                    # print force[2]
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break


        print "="*20 +"go_side"+"="*20
        force_mod = [1,0,0,0,0,1]
        force_toq = [8,0,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        # cmd_str += "\tforce_mode_set_damping(0)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0,0,0,0,0,0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        self.robot.send_program(cmd_str)
        time.sleep(0.5)

        # print self.robot.get_tcp_force()

        while(True):
            try:
                # print "try"
                force = self.robot.get_tcp_force()
                # print "force z : {0}".format(force[2])
                # print force[2]
                if abs(force[0]) > 4:
                    # print force[2]
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break


        print "="*20 +"go_side"+"="*20
        force_mod = [1,1,1,0,0,1]
        force_toq = [0,-8,0,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        # cmd_str += "\tforce_mode_set_damping(0)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0,0,0,0,0,0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        self.robot.send_program(cmd_str)
        time.sleep(0.5)

        # print self.robot.get_tcp_force()

        start_time = time.time()

        while(True):
            try:
                # print "try"
                force = self.robot.get_tcp_force()
                # print "force z : {0}".format(force[2])
                # print force[2]
                if abs(force[2]) > 8 or time.time() - start_time > 20:
                    # print force[2]
                    self.robot.send_program("end_force_mode()")
                    print "end"
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break




    def screw_motion_cus(self, pitch = 0):

        # print "reset"
        # raw_input()


        self.reset()
        time.sleep(1)

        # print "zero_ftsensor"
        # raw_input()


        self.robot.send_program("zero_ftsensor()")
        time.sleep(0.2)
        # print self.robot.get_tcp_force()

        # print "go_down"
        # raw_input()

        # go down
        print "="*20 +"go_down"+"="*20
        force_mod = [1,1,1,1,1,0]
        force_toq = [0,0,-15,0,0,0] 

        cmd_str  = "def go_down():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        # cmd_str += "\tforce_mode_set_damping(0)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0,0,0,0,0,0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        time.sleep(0.1)
        self.robot.send_program(cmd_str)
        time.sleep(0.5)

        # print self.robot.get_tcp_force()

        while(True):
            try:
                # print "try"
                force = self.robot.get_tcp_force()
                # print "force z : {0}".format(force[2])
                # print force[2]
                if abs(force[2]) > 8:
                    # print force[2]
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break


        time.sleep(0.5)

        # spiral    
        print "="*20 +"spiral"+"="*20
        initial = self.robot.getl()
        spiral_cmd = self._get_spiral_cmd(initial)
        self.robot.send_program(spiral_cmd)

        while(True):
            try:
                force = self.robot.get_tcp_force()
                # print force[0], force[1]
                digi = self.robot.get_digital_out(6)
                # print "digi : {0}".format(type(digi))
                if digi > 0: #spiral 실패시
                    self.robot.send_program("set_digital_out(6, False)")
                    return False

                if abs(force[0]) > 35 or abs(force[1]) > 35:
                    # print force[0], force[1]
                    self.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break
        time.sleep(0.1)

        # print "real_insert now?"
        # raw_input()

        

        print "="*20 +"real_insert"+"="*20
        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-30,0,0,0]
        cmd_str  = "def real_insert():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.2, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        self.robot.send_program(cmd_str)

        time.sleep(3)

        # print "reverse now?"
        # raw_input()

        print "="*20 +"spiral_reverse"+"="*20
        initial = self.robot.getl()
        init_pose = copy.deepcopy(initial)
        spiral_cmd = self._get_spiral_cmd(init_pose)
        self.robot.send_program(spiral_cmd)

        while(True):
            if (abs(self.robot.getl()[2]) - abs(init_pose[2])) > 0.1:
                self.robot.send_program("end_force_mode()")
                break


    def part5_motion1(self):
        pre_mid_grip = [0.162392139435, -1.89387526135, -0.950741767883, -1.9049107037, 1.55921697617, 1.54703569412]

        waypoint1 = [-0.22529, -1.9370, -0.90308, -1.8737, 1.5711, 1.3456]

        waypoint2 = [-0.406, -1.954, -0.877, -1.8811, 1.5710, 1.16406]

        waypoint3 = [-0.4849, -1.9116, -0.9383, -1.8636, 1.5710, 1.08604]

        waypoint4 = [-0.641, -2.06462, -0.7144, -1.934, 1.570, 0.9285]

        waypoint4 = [-0.8742, -1.9357, -0.9049, -1.8730, 1.5709, 0.6966]

        goal1 = [-1.42738, -2.15893, -0.56316, -1.9913, 1.57056, 0.14275]

        goal2 = [-1.39765, -1.82456, -1.2239, -1.4413, 1.504668, 0.13185]

        self.gripper_move_and_wait(0)

        self.robot.movej(pre_mid_grip, 1, 1)

        start = self.robot.getl()

        start[2] -= 0.4

        self.robot.movel(start, 1, 1)

        self.gripper_move_and_wait(255)


        start = self.robot.getl()

        start[2] += 0.4

        self.robot.movel(start, 1, 1)

        self.robot.movej(waypoint1, 1, 1)
        self.robot.movej(waypoint2, 1, 1)
        self.robot.movej(waypoint3, 1, 1)
        self.robot.movej(waypoint4, 1, 1)
        self.robot.movej(goal1, 1, 1)
        self.robot.movej(goal2, 1, 1)

        self.robot.send_program("zero_ftsensor()")
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

        self.robot.send_program(cmd_str)

        while(True):
            try:
                force = self.robot.get_tcp_force()

                if force[2] > 10:
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break

        force_mod = [1,0,1,1,0,0]
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

        self.robot.send_program(cmd_str)


    def part5_motion2(self):

        pre_pose1 = [0.15941, -1.9702, -1.8790, -2.4396, 0.20743, 0.032413]

        pre_pose2 = [-0.79945, -2.5119, -1.1388, -2.6832, 0.77916, 0.032761]

        self.gripper_move_and_wait(0)

        self.robot.movej(pre_pose1,1,1)

        self.robot.movej(pre_pose2,1,1)

        start = self.robot.getl()

        start[0] -= 0.1

        self.robot.movel(start, 1, 1)

        self.gripper_move_and_wait(255)

        print "="*20 +"spiral"+"="*20
        initial = self.robot.getl()
        spiral_cmd = self._get_spiral_cmd(initial)
        self.robot.send_program(spiral_cmd)
        time.sleep(0.2)

        while(True):
            try:
                force = self.robot.get_tcp_force()
                # print force[0], force[1]
                digi = self.robot.get_digital_out(6)
                # print "digi : {0}".format(type(digi))
                if digi > 0: #spiral 실패시
                    self.robot.send_program("set_digital_out(6, False)")
                    return False

                if abs(force[0]) > 35 or abs(force[1]) > 35:
                    # print force[0], force[1]
                    self.robot.send_program("end_force_mode()")
                    break

            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break
        time.sleep(0.1)

        self.gripper_move_and_wait(0)

        self.robot.movej(pre_pose2,1,1)

        self.robot.movej(pre_pose1,1,1)


def main():
    rob1 = UrxMotion("192.168.13.101")
    rob2 = UrxMotion("192.168.13.100")

    rob2.part5_motion1()

    rob1.part5_motion2()

    #rob1.gripper_move_and_wait(255)
    #rob2.gripper_move_and_wait(255)

    #start_pose = [0.07926447587050217, -0.5351783236056872, 0.5511638848859853, 3.1363137055133308, -0.000665493946621783, -0.001176621750041618]

    #start_pose = [-0.02697849875540569, -0.46200054319238953, 0.33038970569878207, -3.141160888357369, -7.144981375453567e-05, -4.9131992091015026e-05]

    # print "spiral?"
    # raw_input()
    # rob1.spiral_motion()


    # print "torque start?"
    # raw_input()

    # rob1.screw_motion()

    # rob1.torque_mode([0,0,1,0,0,0], [0,0,-10,0,0,0])
    # rob2.torque_mode([0,0,1,0,0,0], [0,0,-30,0,0,0])

    # print "torque start?"
    # raw_input()

    # initial = rob1.robot.getl()
    # spiral_cmd = rob1._get_spiral_cmd(initial, reverse=True)
    # rob1.robot.send_program(spiral_cmd)

    # print "torque start?"
    # raw_input()

    # rob1.torque_mode([0,0,1,0,0,0], [0,0,-30,0,0,0])

    # print "torque start?"
    # raw_input()

    # rob1.torque_mode([0,0,1,0,0,0], [0,0,10,0,0,0])

    # print "free_drive"
    # raw_input()

    # rob2.screw_motion_cus()
    # rob2.freedrive_mode()

    # rob2._gripper_move(100)

    # rob1.robot.send_program("end_force_mode()")

if __name__ == '__main__':
    main()
