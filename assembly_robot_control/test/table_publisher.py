#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from ur_dashboard_msgs.srv import *
from ur_dashboard_msgs.msg import *
from ur_msgs.msg import *
from ur_msgs.srv import *
from sensor_msgs.msg import *
from std_msgs.msg import *

class Table_pulisher():

	def __init__(self):
		rospy.init_node('table_pulisher', anonymous=True)

		rospy.wait_for_service('/rob2/ur_hardware_interface/set_io')

		self.table = rospy.ServiceProxy('/rob2/ur_hardware_interface/set_io', SetIO)
		self.state = rospy.Subscriber("/rob2/ur_hardware_interface/io_states", IOStates, self.io_state)
		self.state1_update = rospy.Subscriber("/station1/control", String, self.st1_cb)
		self.state2_update = rospy.Subscriber("/station2/control", String, self.st2_cb)
		
		self.pub = rospy.Publisher('/table/joint_states', JointState, queue_size=10)

		self.run = False
		self.pin = [False,False,False,False] 

		self.table1_state = -0.028
		self.table2_state = -0.165

		self.init_flag = 1
		self.count = 0

		rospy.Timer(rospy.Duration(0.1), self.callback)

		print "set"

		# self.table(1,0,0)
		# self.table(1,1,0)
		# self.table(1,3,0)
		# self.table(1,2,0)

	def st1_cb(self, data):
		self.run = False
		if 'up' in data.data :
			self.table1_state = 0.375

			self.table(1,0,1)

			self.table(1,1,1)

		elif 'mid' in data.data:
			self.table1_state = 0


			self.table(1, 0, 0)

			self.table(1, 1, 1)

		elif 'down' in data.data:
			self.table1_state = -0.028

			self.table(1,1,0)

			self.table(1,0,0)

		else:
			print "no mode"

	def st2_cb(self, data):
		self.run = False
		if 'up' in data.data:
			self.table2_state = 0.03

			self.table(1,2,1)

			self.table(1,3,1)

		elif 'mid' in data.data:
			self.table2_state = 0
 
			self.table(1,2,0)
	
			self.table(1,3,1)


		elif 'down' in data.data:
			self.table2_state = -0.165


			self.table(1,3,0)

			self.table(1,2,0)

		else:
			print "no mode"

	def io_state(self, data):
		self.run = data.digital_in_states[0].state
		self.pin = [data.digital_out_states[0].state, data.digital_out_states[1].state, data.digital_out_states[2].state, data.digital_out_states[3].state]

		if self.pin[:2] == [0, 0]:
			self.table1_state = -0.028
		elif self.pin[:2] == [0, 1]:
			self.table1_state = 0
		elif self.pin[:2] == [1, 1]:
			self.table1_state = 0.345

		if self.pin[2:] == [0, 0]:
			self.table2_state = -0.165
		elif self.pin[2:] == [0, 1]:
			self.table2_state = 0
		elif self.pin[2:] == [1, 1]:
			self.table2_state = 0.03

		if self.init_flag:
			self.init_flag = 0
			if self.pin[:2] == [1,0] or self.pin[2:] == [1, 0]:
				self.table(1,0,0)
				self.table(1,1,0)
				self.table(1,3,0)
				self.table(1,2,0)

		

	def callback(self, data):
		self.count = self.count + 1
		jointState = JointState()
		jointState.header.stamp = rospy.Time.now()
		jointState.name = ['table_p_joint1', 'table_p_joint2']
		jointState.position = [self.table1_state, self.table2_state]
		if self.run is True and self.count > 10:
			self.pub.publish(jointState)
			self.count = 0

	def main(self):
		rospy.spin()
		


if __name__ == '__main__':
	try:
		Table_pulisher().main()

	except rospy.ROSInterruptException:
	    pass       