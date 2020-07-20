#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
import moveit_commander
import moveit_msgs.msg

from Assembly_Urx import UrxMotion
# import Assembly_Urx



def main():

    CLOSE = 255
    OPEN = 0

    rospy.init_node('Assembly_example', anonymous=True)

    mg_rob1 = moveit_commander.MoveGroupCommander("rob1_arm")
    mg_rob2 = moveit_commander.MoveGroupCommander("rob2_arm")
    
    urx_rob1 = UrxMotion("192.168.13.101")
    urx_rob2 = UrxMotion("192.168.13.100")

    rob1_pin1_pre_grasp = mg_rob1.get_named_target_values("rob1_pin1_pre_grasp")
    rob1_pin1_grasp = mg_rob1.get_named_target_values("rob1_pin1_grasp")

    rob1_pre_hand_over_pin = mg_rob1.get_named_target_values("rob1_pre_hand_over_pin")
    rob1_hand_over_pin = mg_rob1.get_named_target_values("rob1_hand_over_pin")

    rob2_pre_hand_over_pin = mg_rob2.get_named_target_values("rob2_pre_hand_over_pin")
    rob2_hand_over_pin = mg_rob2.get_named_target_values("rob2_hand_over_pin")

    # move rob1 to rob1_pin1_pre_grasp
    rob1_plan = mg_rob1.plan(rob1_pin1_pre_grasp)
    print "go?" 
    raw_input()
    mg_rob1.execute(rob1_plan)

    # move rob1 to rob1_pin1_grasp
    rob1_plan = mg_rob1.plan(rob1_pin1_grasp)
    print "go?" 
    raw_input()
    mg_rob1.execute(rob1_plan)

    # rob1 close gripper
    urx_rob1.gripper_move_and_wait(CLOSE)

    # move rob1 to rob1_pin1_pre_grasp
    rob1_plan = mg_rob1.plan(rob1_pin1_pre_grasp)
    print "go?" 
    raw_input()
    mg_rob1.execute(rob1_plan)

    # move rob1 and rob2 to pre_hand_over_pin poses
    rob1_plan1 = mg_rob1.plan(rob1_pre_hand_over_pin)
    rob2_plan1 = mg_rob2.plan(rob2_pre_hand_over_pin)
    print "go?" 
    raw_input()
    mg_rob1.execute(rob1_plan1)
    mg_rob2.execute(rob2_plan1)

    # move rob1
    rob1_plan2 = mg_rob1.plan(rob1_hand_over_pin)
    print "go?" 
    raw_input()
    mg_rob1.execute(rob1_plan2)

    # move rob2
    rob2_plan2 = mg_rob2.plan(rob2_hand_over_pin)
    print "go?" 
    raw_input()
    mg_rob2.execute(rob2_plan2)

    # rob2 close gripper
    urx_rob2.gripper_move_and_wait(CLOSE)

    # rob1 open gripper
    urx_rob1.gripper_move_and_wait(OPEN)

    # move rob1
    rob1_plan3 = mg_rob1.plan(rob1_pre_hand_over_pin)
    print "go?" 
    raw_input()
    mg_rob1.execute(rob1_plan3)

    # move rob2
    rob2_plan3 = mg_rob2.plan(rob2_pre_hand_over_pin)
    print "go?" 
    raw_input()
    mg_rob2.execute(rob2_plan3)

if __name__ == '__main__':
    main()