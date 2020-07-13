#!/usr/bin/env python
#-*- coding:utf-8 -*-


import rospy

import Assembly_Mode
from assembly_robot_msgs.msg import Step

def main():
	# 1. 정렬
	# pin - rob1 근처에 모두 위치
	# part - 큰 파트 중 base part 는 rob1, rob2의 workspace가 겹치는 부분에 존재
	#      - 나머지 파트들은 작업 공정 순서 및 크기를 고려하여 배열 했다고 가정
	#	   - 엉덩이 판은 조립 공정 상 특수한 위치에 배치해야 하나 일단 고려하지 않는다. 
	#      - 즉, 정렬이 끝나면 parent part의 위치는 더이상 움직이지 않는다고 가정
	arrange() #정렬 완료되었다 가정
	#

	rospy.init_node('Assembly_example', anonymous=True)
	rospy.Subscriber("Assembly_Step", Step, assembly_cb)
	rospy.spin()


	# 2. 조립 
	# wait for callback - step.msg(맨 처음 정렬한 상태의 데이터)
##############################################
def assembly_cb(msg):
	mode = Assembly_mode()

	for i in range(len(msg.asms)):

		if msg.asms[i].type is 'insert_pin':
			mode.insert_pin(msg.asms[i])

		elif msg.asms[i].type is 'insert_part':
			mode.insert_part(msg.asms[i])

		elif msg.asms[i].type is 'screw':
			mode.screw(msg.asms[i])

		elif msg.asms[i].type is 'flip':
			mode.flip(msg.asms[i])
		
##############################################
# asm.msg 를 나누어 준다


	#임시 함수(윤종헌 Lab.)
###############################################
def arrange():
	print "assembly_start"


if __name__ == '__main__':
 	main()