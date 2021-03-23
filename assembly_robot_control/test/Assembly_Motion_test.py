#!/usr/bin/env python
#-*- coding:utf-8 -*-
# import moveit_commander
# import moveit_msgs.msg

from Assembly_Math_test import *
from move_group_wrapper_test import MoveGroupCommanderWrapper
from std_srvs.srv import *
from sensor_msgs.msg import *
from ur_dashboard_msgs.srv import *
from pandas import Series, DataFrame

from utils.conversions import *

import time

import copy

class Assembly_motion():
    def __init__(self, ros):

        # rospy.init_node('Assembly_Motion', anonymous=True)

        self.mg_rob1 = MoveGroupCommanderWrapper('rob1_arm', 'rob1_real_ee_link')
        self.mg_rob2 = MoveGroupCommanderWrapper('rob2_arm', 'rob2_real_ee_link')
        
        self.mg_rob1.set_planner_id("RRTConnectkConfigDefault")
        self.mg_rob2.set_planner_id("RRTConnectkConfigDefault")

        # self.rob1_client = ros.ServiceProxy('/rob1/ur_hardware_interface/dashboard/play', Trigger)
        # self.rob2_client = ros.ServiceProxy('/rob2/ur_hardware_interface/dashboard/play', Trigger)

        # self.rob1_check = ros.ServiceProxy('/rob1/ur_hardware_interface/dashboard/program_running', IsProgramRunning)
        # self.rob2_check = ros.ServiceProxy('/rob2/ur_hardware_interface/dashboard/program_running', IsProgramRunning)

        self.rob_hand = ros.Publisher('/hand/joint_states', JointState, queue_size=10)
      
        # self.program_running()

        self.init_pose()

        self.rospy = ros
        # time = ros.Time()


    def hand_mode(self, robot, pose): #그리퍼 작동 여부 적용
        if robot is False:
            hand = 'rob1_finger_joint'
        else:
            hand = 'rob2_finger_joint'

        data = JointState()

        data.header.stamp = self.rospy.Time.now()
        data.name.append(hand)
        data.position.append((pose/255.0)*0.8)

        self.rob_hand.publish(data)

    def torque_mode(self, robot ,force_mod, force_toq, tool = False, sleep = 0):
        data = {'robot' : [robot],
            "force_mod"  : [force_mod],
            "force_toq" : [force_toq],
            "tool" :  [tool],
            "sleep" : [sleep]}
        print "+++++++++++++torque_mode+++++++++++++++"
        print DataFrame(data, columns = ['robot', "force_mod", "force_toq", "tool", "sleep"])
    
    def end_mode(self, robot, gripper):
        data = {'robot' : [robot],
            "gripper"  : [force_mod]}
        print "+++++++++++++end_mode+++++++++++++++"
        print DataFrame(data, columns = ['robot', "gripper"])

    def suction_attach(self, robot, force_mod, force_toq):
        data = {'robot' : [robot],
            "force_mod"  : [force_mod],
            "force_toq" : [force_toq]}
        print "+++++++++++++suction_attach+++++++++++++++"
        print DataFrame(data, columns = ['robot', "force_mod", "force_toq"])


    def suction_align(self, robot):
        data = {'robot' : [robot]}
        print "+++++++++++++suction_align+++++++++++++++"
        print DataFrame(data, columns = ['robot'])



    def init_pose(self, robot=None):
        rob1_init_pose = self.mg_rob1.get_named_target_values("rob1_init_pose")
        rob2_init_pose = self.mg_rob2.get_named_target_values("rob2_init_pose")

        if robot is False:
            self.mg_rob1.go(rob1_init_pose)
        elif robot is True:
            self.mg_rob2.go(rob2_init_pose)
        else:
            self.mg_rob1.go(rob1_init_pose)
            self.mg_rob2.go(rob2_init_pose)

    def program_running(self):
        rob1_connect = self.rob1_check()
        rob2_connect = self.rob2_check()
        while rob1_connect.program_running is not True:
            self.rob1_client()
            time.sleep(0.5)
            rob1_connect = self.rob1_check()
            if rob1_connect.program_running is True:
                break

        while rob2_connect.program_running is not True:
            self.rob2_client()
            time.sleep(0.5)
            rob2_connect = self.rob2_check()
            if rob2_connect.program_running is True:
                break

        time.sleep(2)

    
    def pick_up(self, grasp):
        self.group1.go(grasp.pre_grasp)
        self.group1.go(grasp.grasp)
        self.group1.go(grasp.post_grasp)

    #remove
    def trans_convert(self, g_trans, t_trans):
        trans, rot = self.mg_rob1._convert(g_trans, t_trans)
        _trans_list = trans.tolist() + rot.tolist()
        return _trans_list


    def camera_pose(self, robot):
        if robot is False:
            camera_pose_data = self.mg_rob1.get_named_target_values("rob1_camera_pose")
            plan = self.mg_rob1.plan(camera_pose_data)
            self.mg_rob2.execute(plan)
        else:
            camera_pose_data = self.mg_rob2.get_named_target_values("rob2_camera_pose")
            plan = self.mg_rob2.plan(camera_pose_data)
            self.mg_rob2.execute(plan)

    def grab_tool(self, robot, tool, reverse = False):
        if robot is False:
            rob_tool = 'rob1'
            rob = self.mg_rob1
        else:
            rob_tool = 'rob2'
            rob = self.mg_rob2

        if reverse is False:
            self.hand_mode(robot, 0)
        else:
            self.hand_mode(robot, 255)

        tool_pre_grab = rob.get_named_target_values(rob_tool + "_pre_grasp_pose")
        plan = rob.plan(tool_pre_grab) 
        rob.execute(plan, wait=True)


        tool_pre_grab = rob.get_named_target_values(rob_tool + "_"+tool+"_pre_grasp_pose")
        plan = rob.plan(tool_pre_grab) 
        rob.execute(plan, wait=True)


        tool_pre_grab = rob.get_named_target_values(rob_tool + "_"+tool+"_grasp_pose")
        plan = rob.plan(tool_pre_grab) 
        rob.execute(plan, wait=True)

        if reverse is False:
            self.hand_mode(robot, 255)
        else:
            self.hand_mode(robot, 0)

        if tool is not 'tool2':

            tool_pre_grab = rob.get_named_target_values(rob_tool + "_"+tool+"_pre_grasp_pose")
            plan = rob.plan(tool_pre_grab) 
            rob.execute(plan)


        waylist = []

        waypoint = rob.get_current_pose().pose

        waypoint.position.z += 0.3

        waylist.append(copy.deepcopy(waypoint))

        plan, fraction = rob.compute_cartesian_path(waylist, 0.01, 0)

        rob.execute(plan)


    def pick_up_pin(self, robot, pin_name):

        if robot is False:
            rob = "rob1"
            rob_pin_pre_grasp = self.mg_rob1.get_named_target_values("rob1_" + pin_name + "_pre_grasp")
            rob_pin_grasp = self.mg_rob1.get_named_target_values("rob1_" + pin_name + "_grasp")
            mg_rob = self.mg_rob1
        else:
            rob = "rob2"
            rob_pin_pre_grasp = self.mg_rob2.get_named_target_values("rob2_" + pin_name + "_pre_grasp")
            rob_pin_grasp = self.mg_rob2.get_named_target_values("rob2_" + pin_name + "_grasp")
            mg_rob = self.mg_rob2
        #################################################################################
        # 핀 고유의 이름에 맞춘 위치 지점으로 가도록 함
        # rob1_pin1_pre_grasp = self.mg_rob1.get_named_target_values("rob1_"+pin_name+"_pre_grasp")
        # rob1_pin1_grasp = self.mg_rob1.get_named_target_values("rob1_"+pin_name+"_grasp")
        #################################################################################

        mg_rob.go(rob_pin_pre_grasp)
        time.sleep(0.2)

        self.hand_mode(False ,150)


        
        mg_rob.go(rob_pin_grasp)

        self.hand_mode(False, 255)


        waylist = []

        waypoint = mg_rob.get_current_pose().pose

        waypoint.position.z += 0.3

        waylist.append(copy.deepcopy(waypoint))

        plan, fraction = mg_rob.compute_cartesian_path(waylist, 0.01, 0)

        print "plan : {0}".format(plan)
        mg_rob.execute(plan, wait=True)

    def cartestian_move(self, robot, waypoint):
        if robot is False:
            mg_rob = self.mg_rob1

        else:
            mg_rob = self.mg_rob2

        waylist = []

        point = Pose()

        start = mg_rob.get_current_pose().pose

        point.position.x = (start.position.x + waypoint.position.x)/2
        point.position.y = (start.position.y + waypoint.position.y)/2
        point.position.z = (start.position.z + waypoint.position.z)/2

        point.orientation = waypoint.orientation

        waylist.append(point)
        waylist.append(waypoint)

        plan, fraction = mg_rob.compute_cartesian_path(waylist, 0.01, 0)

        mg_rob.execute(plan, wait=True)

    def move_to(self, target_pose, robot):
        if robot is False:
            rob = self.mg_rob1
        else:
            rob = self.mg_rob2

        pose = euler2Pose(target_pose)
        traj = rob.set_pose_target(pose)
        plan = rob.plan(traj)
        rob.execute(plan, wait=True)

    def move_motion(self, grasp_trans, grasp_rot, grasp_offset, robot, c=False, _check = False, collision = True):
        if robot is False:
            rob = self.mg_rob1
        else:
            rob = self.mg_rob2

        success = rob.move_to_grab_part(grasp_trans, grasp_rot, grasp_offset, c, _check, collision)
        return success


    def attach(self, robot, part, file_name):
        if robot is False:
            rob = self.mg_rob1
            hand = 'rob1'
        else:
            rob = self.mg_rob2
            hand = 'rob2'

        rob.attach(hand, part, file_name)


    def dettach(self, robot, part):
        if robot is False:
            rob = self.mg_rob1
            hand = 'rob1'
        else:
            rob = self.mg_rob2
            hand = 'rob2'

        rob.dettach(hand, part)


    def move_current_to(self,x,y,z,robot):
        if robot is False:
            rob = self.mg_rob1
        else:
            rob = self.mg_rob2

        c_pose = copy.deepcopy(rob.get_current_pose().pose)
        c_pose.position.x += x
        c_pose.position.y += y

        traj = rob.set_pose_target(c_pose)


        plan = rob.plan(traj)

        rob.execute(plan, wait=True)


    def move_current_up(self, z, robot):
        if robot is False:
            rob = self.mg_rob1
        else:
            rob = self.mg_rob2

        c_pose = copy.deepcopy(rob.get_current_pose().pose)
        c_pose.position.z += z

        traj = rob.set_pose_target(c_pose)
        plan = rob.plan(traj)

        rob.execute(plan, wait=True)

    def current_pose(self, robot, reset = False, pose = None):
        rob = self.mg_rob1 if robot is False else self.mg_rob2

        if reset is False:
            return rob.get_current_pose().pose
        else:
            traj = rob.set_pose_target(pose)
            plan = rob.plan(traj)
            rob.execute(plan, wait=True)

    def attach_motion(self, robot, joint):
        rob = self.mg_rob1 if robot is False else self.mg_rob2

        joint_value = rob.get_current_joint_values()

        joint_value = joint

        rob.set_joint_value_target(joint_value)

        plan = rob.plan()

        rob.execute(plan, wait=True)
            

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
        self.hand_mode(True, 255)
        # rob1 open gripper
        self.hand_mode(False, 0)

        self.program_running()

         # move rob1 and rob2 to pre_hand_over_pin poses
        self.mg_rob1.go(rob1_pre_hand_over_pin)
        self.mg_rob2.go(rob2_pre_hand_over_pin)

        # NotImplementedError
        

    def hold_assistant(self, grasp_trans, grasp_rot, grasp_offset, robot, c=False):
        if robot is False:
            rob = self.mg_rob1
        else:
            rob = self.mg_rob2

        self.hand_mode(robot, 0)

        r = rob.move_to_hold_part(grasp_trans, grasp_rot, grasp_offset, c)
        if r is False: return r

        self.hand_mode(robot, 255)


        
    def gripper_control(self, robot, target):
        self.hand_mode(robot, target)



 
    def sprial_pin(self, robot = False, pitch = 0, gripper = 0):
       
        print "*****************spiral*****************"

        self.hand_mode(robot, gripper)

        return True

    def screw_motion(self, pitch = 0, robot = False):


        print "go down now? \n\ncheck tool?"
        raw_input()        

        return True
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


  

# if __name__ == '__main__':
#     main()