#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
import moveit_commander
import moveit_msgs.msg
from Assembly_Urx_test import UrxMotion

class Assembly_motion():
    def __init__(self):

        rospy.init_node('Assembly_Motion', anonymous=True)

        self.mg_rob1 = moveit_commander.MoveGroupCommander("rob1_arm")
        self.mg_rob2 = moveit_commander.MoveGroupCommander("rob2_arm")
        self.urx_rob1 = UrxMotion("192.168.13.101")
        self.urx_rob2 = UrxMotion("192.168.13.100")
    
    def pick_up(self, grasp):
        self.group1.go(grasp.pre_grasp)
        self.group1.go(grasp.grasp)
        self.group1.go(grasp.post_grasp)


    def pick_up_pin(self, pin_name):

        rob1_pin1_pre_grasp = self.mg_rob1.get_named_target_values("rob1_pin1_pre_grasp")
        rob1_pin1_grasp = self.mg_rob1.get_named_target_values("rob1_pin1_grasp")

        #################################################################################
        # 핀 고유의 이름에 맞춘 위치 지점으로 가도록 함
        # rob1_pin1_pre_grasp = self.mg_rob1.get_named_target_values("rob1_"+pin_name+"_pre_grasp")
        # rob1_pin1_grasp = self.mg_rob1.get_named_target_values("rob1_"+pin_name+"_grasp")
        #################################################################################

        self.mg_rob1.go(rob1_pin1_pre_grasp)
        self.urx_rob1.gripper_move_and_wait(0)
        self.mg_rob1.go(rob1_pin1_grasp)
        self.urx_rob1.gripper_move_and_wait(255)
        self.mg_rob1.go(rob1_pin1_pre_grasp)

    def move_to(self, target_pose):
        print "move_to"
        #수정전 임시 출력 


    def hand_over_pin(self):
        # 넘기기 작업만 사용 판단은 상위에서
        rob1_pre_hand_over_pin = self.mg_rob1.get_named_target_values("rob1_pre_hand_over_pin")
        rob1_hand_over_pin = self.mg_rob1.get_named_target_values("rob1_hand_over_pin")

        rob2_pre_hand_over_pin = self.mg_rob2.get_named_target_values("rob2_pre_hand_over_pin")
        rob2_hand_over_pin = self.mg_rob2.get_named_target_values("rob2_hand_over_pin")

        # move rob1 and rob2 to pre_hand_over_pin poses
        self.mg_rob1.go(rob1_pre_hand_over_pin)
        self.mg_rob2.go(rob2_pre_hand_over_pin)

        # move rob1 and rob2 to hand_over_pin poses
        self.mg_rob1.go(rob1_hand_over_pin)
        self.mg_rob2.go(rob2_hand_over_pin)

        # rob2 close gripper
        self.urx_rob2.gripper_move_and_wait(255)
        # rob1 open gripper
        self.urx_rob1.gripper_move_and_wait(0)

         # move rob1 and rob2 to pre_hand_over_pin poses
        self.mg_rob1.go(rob1_pre_hand_over_pin)
        self.mg_rob2.go(rob2_pre_hand_over_pin)

        # NotImplementedError
        

    def hold_assistant(self, part_name, robot):
        if robot is 1:
            rob = self.rob1
        else:
            rob = self.rob2

        pre_hold_part = rob.get_named_target_values("rob"+str(robot)+"_pre_hold_"+part_name)
        hold_part = rob.get_named_target_values("rob"+str(robot)+"_hold_"+part_name)

        self.urx_rob1.gripper_move_and_wait(0)
        rob.go(pre_hold_part)
        rob.go(hold_part)
        self.urx_rob2.gripper_move_and_wait(255)




 
    def sprial_pin(self, parent_name , robot = 1):
        # spiral motion을 진행하면서 pin을 insert 하는 작업
        # force값을 받아서 pin이 insert가 되었는지 아닌지 확인
        # insert가 되면 모션 중지하고 True를 반환
        # motion을 끝까지 진행하였는데도 insert가 안되었으면 False를 반환
        #####
        if robot is 1:
            self.rob1.sprial_motion()
        else:
            self.rob2.spiral_motion()
        # return is_inserted




def main():
    am = Assembly_motion()
    #am.grab_pin() # grab_pin은 무조건 rob1이
    # insert pin을 할 hole의 위치에 따라서 다른 로봇 팔이 fix할 위치도 결정되어 있음
    # target_pose1이 어느 로봇에 더 기까운지로 spiral motion을 할 로봇이 결정
    #am.fix_part()
    #am.insert_spiral_motion(target_pose1)
    am.pick_up_pin("1")
    am.hand_over_pin()


  

if __name__ == '__main__':
    main()