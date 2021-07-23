#-*- coding:utf-8 -*-
import time
import urx
import math as m
import numpy as np
import copy
import random

robot = urx.URRobot("192.168.13.155", use_rt=True) 

time.sleep(2)


# msg = 'vgp_grip(channel_select=[True, True, True, True], channel_require=[True, True, True, True])'


# robot.send_program("vgp_grip(channel_select = [True,True,True,True], channel_require=[True,True,True,True], vacuum = 20, alert = False, tool_index=0)")

raw_input("go? | enter")

robot.send_program("vgp_grip(channel_select=[True, True, True, True], channel_require=[True, True, True, True])")

raw_input("go?")  

cmd_str  = "def go_down():\n"
cmd_str += "\twhile (True)\n"
cmd_str += "\t\ton_return = vgp_grip(channel_select = [True,True,True,True], channel_require=[True,True,True,True], vacuum = 20, alert = False, tool_index=0)\n"
cmd_str += "\tend\n"
cmd_str += "end\n"

robot.send_program(cmd_str)