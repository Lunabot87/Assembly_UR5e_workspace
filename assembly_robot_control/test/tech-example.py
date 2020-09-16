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
        #self.robot.send_program(self._set_gripper())
        time.sleep(3)

    def reset(self):
        connected = False 
        while not connected:      
            try:         
                self.robot = urx.URRobot(self.robot_ip, use_rt=True)        
                connected = True    
            except:     
                time.sleep(1)  

    def _format_move(self, command, tpose, acc, vel, radius=0, prefix=""):
        tpose = [round(i, 6) for i in tpose]
        tpose.append(acc)
        tpose.append(vel)
        tpose.append(radius)
        return "{}({}[{},{},{},{},{},{}], a={}, v={}, r={})".format(command, prefix, *tpose)  

    def _get_spiral_cmd(self, initial):
        vel = 0.5
        acc = 0.25
        radius = 0.0005

        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-7,0,0,0] 

        # spiral motion
        R = 0.006 #0.003
        revolution = 8 #4
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
        msg += "\t\t\tset_digital_out(1, True)\n"
        msg += "\t\t\tbreak\n"
        msg += "\t\telse:\n"
        msg += "\t\t\trq_pos_1 = rq_pos\n"
        msg += "\t\tend\n"
        msg += "\tend\n"
        msg += "end\n"

        return msg


    def gripper_move_and_wait(self ,pos):
        robot = self.robot

        robot.send_program(self._gripper_move(pos))

        while True:
            if self.robot.get_digital_out(1) != 0:
                self.robot.send_program("set_digital_out(1, False)")
                print "gripper_move_and_wait complete"
                break
            else:
                continue

    def check_motion(self, num, move):

        print "="*20 +"check"+"="*20      
        force_mod = [1,1,1,1,1,1]
        force_toq = [0,0,-8,0,0,num]
        cmd_str  = "def real_insert():"
        cmd_str += "\tforce_mode_set_damping(0.005)\n"
        cmd_str += "\twhile (True):\n"
        cmd_str += "\t\tforce_mode(p[0.0,0.0,0.0,0.0,0.0,0.0], "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.2, 0.17, 0.17, 0.17])\n"
        cmd_str += "\t\tsync()\n"
        cmd_str += "\tend\n"
        cmd_str += "end\n"
        self.robot.send_program(cmd_str)
        p_i = self.robot.getl()
        while(True):
            try:
                force = self.robot.get_tcp_force()
                p_c = self.robot.getl()
                print  force[5]
                if abs(force[5]) > 10 or np.linalg.norm(np.array(p_c)-np.array(p_i) > move):
                    print force[5]
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break
        time.sleep(0.1)


    def spiral_motion(self):


        start_pose = [0.20585066275426037, -0.42743467242629213, 0.4429043519861413, 2.328130001856936, -2.0341398615611057, 0.08922805799939844]


        
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

        print "move"
        raw_input()

        self.robot.movel(start_pose, wait=True)

        while(True):
            current = self.robot.getl()
            dist = np.linalg.norm(np.array(current)-np.array(start_pose))
            if dist < 0.001:
                break
        self.reset()
        time.sleep(1)

        self.robot.send_program("zero_ftsensor()")
        time.sleep(0.2)
        # print self.robot.get_tcp_force()

        print "down"
        raw_input()

        # go down
        print "="*20 +"go_down"+"="*20
        force_mod = [0,0,1,0,0,0]
        force_toq = [0,0,-1,0,0,0] 

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
                #print force[2]
                # print force[2]
                if abs(force[2]) > 1.5:
                    #print force[2]
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break


        time.sleep(0.1)

        print "spiral"
        raw_input()

        # spiral    
        print "="*20 +"spiral"+"="*20
        initial = self.robot.getl()
        spiral_cmd = self._get_spiral_cmd(initial)
        self.robot.send_program(spiral_cmd)

        while(True):
            try:
                force = self.robot.get_tcp_force()
                print force[0], force[1]
                if abs(force[0]) > 10 or abs(force[1]) > 10:
                    print force[0], force[1]
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break
        time.sleep(0.1)

        print "check"
        raw_input()

        # check
        self.check_motion(2, 0.0002)
        self.check_motion(-2, 0.0002)

        print "juggle"
        raw_input()

        # check
        for i in range(5):
            self.check_motion(4, 0.012)
            self.check_motion(-2, 0.003)


        print "insert"
        raw_input()


        # real insert
        print "="*20 +"real_insert"+"="*20
        force_mod = [1,1,1,0,0,0]
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
                    print force[2]
                    time.sleep(1)
                    self.robot.send_program("end_force_mode()")
                    break
            except KeyboardInterrupt:
                self.robot.send_program("end_force_mode()")
                break

        ########################################################################

        # self.gripper_move_and_wait(0)


        # post_pose = self.robot.getl()
        # post_pose[2] += 0.1
        # self.robot.movel(post_pose)

        # while(True):
        #     current = self.robot.getl()
        #     dist = np.linalg.norm(np.array(current)-np.array(post_pose))
        #     if dist < 0.001:
        #         break


def main():
    rob1 = UrxMotion("192.168.13.101")
    rob2 = UrxMotion("192.168.13.100")

    #rob1.gripper_move_and_wait(255)
    #rob2.gripper_move_and_wait(255)

    #start_pose = [0.07926447587050217, -0.5351783236056872, 0.5511638848859853, 3.1363137055133308, -0.000665493946621783, -0.001176621750041618]

    #start_pose = [-0.02697849875540569, -0.46200054319238953, 0.33038970569878207, -3.141160888357369, -7.144981375453567e-05, -4.9131992091015026e-05]

    # rob1.spiral_motion()
    print "start?"
    raw_input()

    rob1.check_motion(-3, 0.0005 )
    rob1.check_motion(3, 0.0010 )
    print "start?"
    raw_input()
    rob1.check_motion(-3, 0.0006 + 0.0005 )
    rob1.check_motion(3, 0.0006*2 )
    print "start?"
    raw_input()
    rob1.check_motion(-3, 0.0007 + 0.0006 )
    rob1.check_motion(3, 0.0007*2 )
    print "start?"
    raw_input()
    rob1.check_motion(-3, 0.0008 + + 0.0007)
    rob1.check_motion(3, 0.0008*2 )
    print "start?"
    raw_input()
    rob1.check_motion(-3, 0.0009 + 0.0008 )
    rob1.check_motion(3, 0.0009*2 )
    # print rob1.robot.getl()

if __name__ == '__main__':
    main()