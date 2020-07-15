import time
import urx
import logging
import sys
import math as m
import numpy as np
import copy
import random

sys.path.append('../../assembly_robot_gripper_control/scripts')
from Robotiq_2f_Gripper import Robotiq_2f_Gripper

start_pose = [0.07926447587050217, -0.5351783236056872, 0.5511638848859853, 3.1363137055133308, -0.000665493946621783, -0.001176621750041618]

def _format_move(command, tpose, acc, vel, radius=0, prefix=""):
    tpose = [round(i, 6) for i in tpose]
    tpose.append(acc)
    tpose.append(vel)
    tpose.append(radius)
    return "{}({}[{},{},{},{},{},{}], a={}, v={}, r={})".format(command, prefix, *tpose)  

def get_spiral_cmd(initial):
    vel = 0.5
    acc = 0.25
    radius = 0.0005

    force_mod = [0,0,1,0,0,0]
    force_toq = [0,0,7,0,0,0] 

    # spiral motion
    R = 0.006 #0.003 #0.006
    revolution = 8 #4 #8 
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
    cmd_str += "\t\t\tforce_mode(tool_pose(), "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
    cmd_str += "\t\t\tsync()\n"
    cmd_str += "\t\tend\n"
    cmd_str += "\tend\n"
    cmd_str += "\tglobal thrd = run Thread_1()\n"    

    for idx, pose in enumerate(spiral_list):
        if idx == (len(spiral_list) - 1):
            radius = 0
        test_cmd = _format_move("movej", pose, acc, vel, radius, prefix="p") + "\n"
        cmd_str += test_cmd
    
    cmd_str += "\tend_force_mode()\n"
    cmd_str += "\tkill thrd\n"
    cmd_str += "end\n"

    return cmd_str

# def wait_for_move()

def main():
    global start_pose

    #rob1grip = Robotiq_2f_Gripper("192.168.13.101") #action_gripper(action_pos, wait_time = 2) open = 0, close = 255
    rob1 = urx.Robot("192.168.13.101", use_rt=True)

    # move to initial pose
    error = 0.006
    random_error_x = error*random.uniform(-1, 1)
    random_error_y = error*random.uniform(-1, 1)
    # random_error_x = 0.00140919120041
    # random_error_y = 0.0052389419886
    start_pose[0] += -error #random_error_x
    start_pose[1] += 0 #random_error_y
    print "error_x={}, error_y={}".format(random_error_x, random_error_y)
    rob1.movel(start_pose, wait=True)

    while(True):
        current = rob1.getl()
        dist = np.linalg.norm(np.array(current)-np.array(start_pose))
        if dist < 0.001:
            break
    
    rob1.send_program("zero_ftsensor()")
    time.sleep(0.2)
    
    # go down
    print "="*20 +"go_down"+"="*20
    force_mod = [0,0,1,0,0,0]
    force_toq = [0,0,1,0,0,0] 

    cmd_str  = "def go_down():"
    cmd_str += "\tforce_mode_set_damping(0.005)\n"
    cmd_str += "\twhile (True):\n"
    cmd_str += "\t\tforce_mode(tool_pose(), "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
    cmd_str += "\t\tsync()\n"
    cmd_str += "\tend\n"
    cmd_str += "end\n"
    time.sleep(0.1)
    rob1.send_program(cmd_str)

    while(True):
        try:
            force = rob1.get_tcp_force()
            # print force[2]
            if force[2] > 3:
                print force[2]
                rob1.send_program("end_force_mode()")
                break
        except KeyboardInterrupt:
            rob1.send_program("end_force_mode()")
            break
    time.sleep(0.1)

    # spiral    
    print "="*20 +"spiral"+"="*20
    initial = rob1.getl()
    spiral_cmd = get_spiral_cmd(initial)
    # print spiral_cmd
    # rob1.send_program("force_mode_set_damping(0)")
    # time.sleep(0.1)
    rob1.send_program(spiral_cmd)

    while(True):
        try:
            force = rob1.get_tcp_force()
            # print force[0], force[1]
            if abs(force[0]) > 10 or abs(force[1]) > 10:
                print force[0], force[1]
                rob1.send_program("end_force_mode()")
                break
        except KeyboardInterrupt:
            rob1.send_program("end_force_mode()")
            break
    time.sleep(0.1)


    # spiral check

    # real insert
    print "="*20 +"real_insert"+"="*20
    force_mod = [1,1,1,0,0,0]
    force_toq = [0,0,50,0,0,0]
    cmd_str  = "def real_insert():"
    cmd_str += "\tforce_mode_set_damping(0.005)\n"
    cmd_str += "\twhile (True):\n"
    cmd_str += "\t\tforce_mode(tool_pose(), "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.2, 0.17, 0.17, 0.17])\n"
    cmd_str += "\t\tsync()\n"
    cmd_str += "\tend\n"
    cmd_str += "end\n"
    rob1.send_program(cmd_str)

    while(True):
        try:
            force = rob1.get_tcp_force()
            # print force[2]
            if force[2] > 50:
                print force[2]
                time.sleep(1)
                rob1.send_program("end_force_mode()")
                break
        except KeyboardInterrupt:
            rob1.send_program("end_force_mode()")
            break


if __name__ == '__main__':
    main()