#-*- coding:utf-8 -*-

import moveit_commander
import moveit_msgs.msg
from Assembly_Urx import UrxMotion
from tf import *

class Assembly_motion():
    def __init__(self):
        self.mg_rob1 = moveit_commander.MoveGroupCommander("rob1")
        self.mg_rob2 = moveit_commander.MoveGroupCommander("rob2")
        self.urx_rob1 = UrxMotion("192.168.13.101")
        self.urx_rob2 = UrxMotion("192.168.13.100")
        listener = TransformListener()
    
    def pick_up(self, grasp):
        self.group1.go(grasp.pre_grasp)
        self.group1.go(grasp.grasp)
        self.group1.go(grasp.post_grasp)

    def move_to(self, target_pose):
        print "move_to"
        #수정전 임시 출력 

    def hand_over_pin(self, target_name): #pose or tf
        rob1_dt, rob1_rpy = listener.lookupTransform('/rob1_real_base_link', target_name, rospy.Time(0))
        rob2_dt, rob2_rpy = listener.lookupTransform('/rob2_real_base_link', target_name, rospy.Time(0))

        if np.linalg.norm(np.array(rob1_dt)) < np.linalg.norm(np.array(rob2_dt)):
            #rob1가 작업 
        else:
            #rob2이 작업

        # NotImplementedError
        
    def sprial_motion(self):
        # spiral motion을 진행하면서 pin을 insert 하는 작업
        # force값을 받아서 pin이 insert가 되었는지 아닌지 확인
        # insert가 되면 모션 중지하고 True를 반환
        # motion을 끝까지 진행하였는데도 insert가 안되었으면 False를 반환
        #####
        return is_inserted

def main():
    am = Assembly_motion()
    am.grab_pin() # grab_pin은 무조건 rob1이
    # insert pin을 할 hole의 위치에 따라서 다른 로봇 팔이 fix할 위치도 결정되어 있음
    # target_pose1이 어느 로봇에 더 기까운지로 spiral motion을 할 로봇이 결정
    am.fix_part()
    am.insert_spiral_motion(target_pose1)



  

if __name__ == '__main__':
    main()