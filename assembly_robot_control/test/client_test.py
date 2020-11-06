#!/usr/bin/env python
#-*- coding:utf-8 -*-


import rospy
import time
from assembly_robot_msgs.srv import asm_Srv
from assembly_robot_msgs.msg import *


class Interface_for_Robot():
    def __init__(self, compt=False):

    	rospy.init_node('Asm_t', anonymous=True)
        self.client_for_robot = rospy.ServiceProxy("/to_RobotControl", asm_Srv)
        rospy.wait_for_service("/to_RobotControl")



    def service_test(self):
        # parent = Asm_test(["PART6_1"], ["PART6_1-hole_3"])
        # child = Asm_test(["C101350_3"], ["C101350_3-pin_1_spare"])
        # resp = self.client_for_robot("insert", parent, child)
        # time.sleep(1)

        # parent = Asm_test(["PART6_1"], ["PART6_1-hole_1"])
        # child = Asm_test(["C101350_4"], ["C101350_4-pin_1_spare"])
        # resp = self.client_for_robot("insert", parent, child)
        # time.sleep(1)



        # parent = Asm_test(["chair_part6"], ["hole6-1"])
        # child = Asm_test(["c101350_1"], ['c101350_1'])
        # resp = self.client_for_robot("insert", parent, child)
        # # print "next?"
        # # raw_input()

        # parent = Asm_test(["chair_part6"], ["hole6-2"])
        # child = Asm_test(["c101350_2"], ['c101350_2'])
        # resp = self.client_for_robot("insert", parent, child)
        # # print "next?"
        # raw_input()

        # parent = Asm_test(["chair_part6"], ["hole6-3"])
        # child = Asm_test(["c101350_1"], ['c101350_1'])
        # resp = self.client_for_robot("insert", parent, child)
        # # print "next?"
        # # raw_input()

        # parent = Asm_test(["chair_part6"], ["hole6-4"])
        # child = Asm_test(["c101350_1"], ['c101350_1'])
        # resp = self.client_for_robot("insert", parent, child)
        # print "next?"
        # raw_input()

        # parent = Asm_test(["chair_part6"], ["hole6-5"])
        # child = Asm_test(["c101350_1"], ['c101350_1'])
        # resp = self.client_for_robot("insert", parent, child)
        # print "next?"
        # raw_input()

        # parent = Asm_test(["chair_part6"], ["hole6-6"])
        # child = Asm_test(["c101350_1"], ['c101350_1'])
        # resp = self.client_for_robot("insert", parent, child)
        # print "next?"
        # raw_input()

        # parent = Asm_test(["chair_part6"], ["hole6-7"])
        # child = Asm_test(["c101350_1"], ['c101350_1'])
        # resp = self.client_for_robot("insert", parent, child)
        # print "next?"
        # raw_input()

        # parent = Asm_test(["chair_part6"], ["hole6-1", "hole6-2"])
        # child = Asm_test(["chair_part2"], ["hole2-1", "hole2-2"])
        # resp = self.client_for_robot("insert", parent, child)
        # # print "next?"
        # raw_input()

        # parent = Asm_test(["chair_part2"], ["hole2-3"])
        # child = Asm_test(["c101350_1"], ['c101350_1'])
        # resp = self.client_for_robot("insert", parent, child)
        # print "next?"
        # raw_input()

        # parent = Asm_test(["chair_part2"], ["hole2-4"])
        # child = Asm_test(["c101350_1"], ['c101350_1'])
        # resp = self.client_for_robot("insert", parent, child)
        # print "next?"
        # raw_input()

        # parent = Asm_test(["chair_part6"], ["hole6-3", "hole6-4"])
        # child = Asm_test(["chair_part3"], ["hole3-3", "hole3-4"])
        # resp = self.client_for_robot("insert", parent, child)
        # # print "next?"
        # # raw_input()

        # parent = Asm_test(["chair_part3"], ["hole3-1"])
        # child = Asm_test(["c101350_3"], ['c101350_3'])
        # resp = self.client_for_robot("insert", parent, child)
        # # print "next?"
        # # raw_input()

        # parent = Asm_test(["chair_part3"], ["hole3-2"])
        # child = Asm_test(["c101350_4"], ['c101350_4'])
        # resp = self.client_for_robot("insert", parent, child)
        # # print "next?"
        # raw_input()


        # parent = Asm_test(["chair_part6"], ["hole6-5", "hole6-6", 'hole6-7'])
        # child = Asm_test(["chair_part4"], ["hole4-1", "hole4-2", 'hole4-6'])
        # resp = self.client_for_robot("insert", parent, child)
        # print "next?"
        # raw_input()

        # parent = Asm_test(["PART6_1"], ["PART2_1-hole_1"])
        # child = Asm_test(["C101350_1"], ["C101350_1-pin_1"])
        # resp = self.client_for_robot("insert", parent, child)
        # time.sleep(1)

        # parent = Asm_test(["PART6_1"], ["PART2_1-hole_5"])
        # child = Asm_test(["C101350_2"], ["C101350_2-pin_1"])
        # resp = self.client_for_robot("insert", parent, child)
        # time.sleep(1)


if __name__ == '__main__':
    aa = Interface_for_Robot()
    aa.service_test()