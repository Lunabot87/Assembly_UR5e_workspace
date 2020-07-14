import time
import urx
import logging
import sys
import math as m
import numpy as np
import copy

sys.path.append('../../assembly_robot_gripper_control/scripts')
from Robotiq_2f_Gripper import Robotiq_2f_Gripper

start_pose = [0.15697152980980725, -0.4958977011271538, 0.24435886093454873, 
            -2.221022989086017, -2.2214727795467892, 0.0004896852898812067]

def _format_move(command, tpose, acc, vel, radius=0, prefix=""):
    tpose = [round(i, 6) for i in tpose]
    tpose.append(acc)
    tpose.append(vel)
    tpose.append(radius)
    return "{}({}[{},{},{},{},{},{}], a={}, v={}, r={})".format(command, prefix, *tpose)  

def get_spiral_cmd(initial):
    vel = 0.3
    acc = 0.1
    radius = 0.0005

    force_mod = [0,0,1,0,0,0]
    force_toq = [0,0,1,0,0,0] 

    # spiral motion
    R = 0.01
    dth = m.pi/30
    dr = R*0.005
    th_array = np.arange(0, 6*m.pi+dth, step=dth)
    r_array = np.arange(0, R, step=dr)
    # print th_array
    spiral_list = []

    # get initial pose
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
    cmd_str += "\tthread Thread_1():\n"
    cmd_str += "\t\tforce_mode_set_damping(0)\n"
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

def main():
    global start_pose

    #rob1grip = Robotiq_2f_Gripper("192.168.13.101") #action_gripper(action_pos, wait_time = 2) open = 0, close = 255
    rob1 = urx.Robot("192.168.13.101", use_rt=True)
    
    # move to initial pose
    # rob1.movel(start_pose, wait=False)


    # go down
    force_mod = [0,0,1,0,0,0]
    force_toq = [0,0,1,0,0,0] 

    cmd_str  = "def go_down():"
    cmd_str += "\twhile (True):\n"
    cmd_str += "\t\tforce_mode_set_damping(0.005)\n"
    cmd_str += "\t\tforce_mode(tool_pose(), "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
    cmd_str += "\t\tsync()\n"
    cmd_str += "\tend\n"
    cmd_str += "end\n"
    rob1.send_program(cmd_str)

    while(True):
        try:
            force = rob1.get_tcp_force()
            # print force[2]
            if force[2] > 3:
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
    rob1.send_program(spiral_cmd)

    while(True):
        try:
            force = rob1.get_tcp_force()
            print force[0], force[1]
            if abs(force[0]) > 3 or abs(force[1]) > 3:
                rob1.send_program("end_force_mode()")
                break
        except KeyboardInterrupt:
            rob1.send_program("end_force_mode()")
            break
    time.sleep(0.1)

    # real insert
    print "="*20 +"real_insert"+"="*20
    force_mod = [1,1,1,0,0,0]
    force_toq = [0,0,50,0,0,0]
    cmd_str  = "def real_insert():"
    cmd_str += "\twhile (True):\n"
    cmd_str += "\t\tforce_mode_set_damping(0.005)\n"
    cmd_str += "\t\tforce_mode(tool_pose(), "+str(force_mod) +"," + str(force_toq) +", 2, [0.1, 0.1, 0.15, 0.17, 0.17, 0.17])\n"
    cmd_str += "\t\tsync()\n"
    cmd_str += "\tend\n"
    cmd_str += "end\n"
    rob1.send_program(cmd_str)

    while(True):
        try:
            force = rob1.get_tcp_force()
            print force[2]
            if force[2] > 30:
                rob1.send_program("end_force_mode()")
                break
        except KeyboardInterrupt:
            rob1.send_program("end_force_mode()")
            break


if __name__ == '__main__':
    main()