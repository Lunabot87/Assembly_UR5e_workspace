#-*- coding:utf-8 -*-

import moveit_commander
import moveit_msgs.msg
from Assembly_Urx import UrxMotion

class Assembly_motion():
    def __init__(self):
        # self.mg_rob1 = moveit_commander.MoveGroupCommander("rob1")
        # self.mg_rob2 = moveit_commander.MoveGroupCommander("rob2")
        self.urx_rob1 = UrxMotion("192.168.13.101")
        self.urx_rob2 = UrxMotion("192.168.13.100")
    
    def pick_up(self, grasp):
        self.group1.go(grasp.pre_grasp)
        self.group1.go(grasp.grasp)
        self.group1.go(grasp.post_grasp)

    def move_to(self, target_pose):
        print "move_to"
        #수정전 임시 출력 

    def hand_over_pin(self):
        NotImplementedError

    def sprial_motion(self):
        # spiral motion을 진행하면서 pin을 insert 하는 작업
        # force값을 받아서 pin이 insert가 되었는지 아닌지 확인
        # insert가 되면 모션 중지하고 True를 반환
        # motion을 끝까지 진행하였는데도 insert가 안되었으면 False를 반환
        #####
        return is_inserted

def main():
    rob1 = UrxMotion("192.168.13.101")

    exchange_pin_pose_rob1 = [0.006926903133457668, -0.522865513543282, 0.8865461745794542, 1.208677247239153, 1.209773090726131, -1.2086506031939017]
    exchange_pin_pose_rob2 = [-0.018465001144859182, -0.5537246715577478, 0.8888033542107026, 1.2091777727575899, -1.209075954976332, 1.2095379430867241]

    rob1.robot.send_program("movej(p[0.006926903133457668, -0.522865513543282, 0.8865461745794542, 1.208677247239153, 1.209773090726131, -1.2086506031939017])")


if __name__ == '__main__':
    main()