import urx
import time

robot = urx.URRobot("192.168.13.100", use_rt = True)

pre_mid_grip = [0.162392139435, -1.89387526135, -0.950741767883, -1.9049107037, 1.55921697617, 1.54703569412]

mid_grip  =	[0.0404849052429, -2.13945736508, -1.7180891037, -0.856333093052, 1.57082843781, 1.39017820358]

#0.161204338074, -2.09000219921, -1.75568675995, -0.903780297642, 1.55892848969, 1.54784059525

waypoint1 = [-0.22529, -1.9370, -0.90308, -1.8737, 1.5711, 1.3456]

waypoint2 = [-0.406, -1.954, -0.877, -1.8811, 1.5710, 1.16406]

waypoint3 = [-0.4849, -1.9116, -0.9383, -1.8636, 1.5710, 1.08604]

waypoint4 = [-0.641, -2.06462, -0.7144, -1.934, 1.570, 0.9285]

waypoint4 = [-0.8742, -1.9357, -0.9049, -1.8730, 1.5709, 0.6966]

goal1 = [-1.42738, -2.15893, -0.56316, -1.9913, 1.57056, 0.14275]

# goal2 = [-1.442, -1.940 ,-1.070, -1.489 ,1.553 ,0.0807]

goal2 = [-1.39765, -1.82456, -1.2239, -1.4413, 1.504668, 0.13185]


robot.movej(pre_mid_grip, 1, 1)

start = robot.getl()

start[2] -= 0.4

robot.movel(start, 1, 1)

print "gipper"
raw_input()


start = robot.getl()

start[2] += 0.4

robot.movel(start, 1, 1)

print "gipper"
raw_input()

robot.movej(waypoint1, 1, 1)
robot.movej(waypoint2, 1, 1)
robot.movej(waypoint3, 1, 1)
robot.movej(waypoint4, 1, 1)
robot.movej(goal1, 1, 1)
robot.movej(goal2, 1, 1)

robot.send_program("zero_ftsensor()")
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

robot.send_program(cmd_str)

while(True):
    try:
        force = robot.get_tcp_force()

        if force[2] > 10:
            robot.send_program("end_force_mode()")
            break
    except KeyboardInterrupt:
        robot.send_program("end_force_mode()")
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

robot.send_program(cmd_str)