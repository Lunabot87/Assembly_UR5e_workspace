#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from ur_dashboard_msgs.srv import *
from ur_dashboard_msgs.msg import *
from ur_msgs.msg import *
from ur_msgs.srv import *
from sensor_msgs.msg import *
from std_msgs.msg import *

class Tool_pulisher():

	def __init__(self):
		rospy.init_node('tool_pulisher', anonymous=True)

		rospy.wait_for_service('/rob2/ur_hardware_interface/set_io')

		self.io_pub = rospy.ServiceProxy('/rob2/ur_hardware_interface/set_io', SetIO)
		self.state = rospy.Subscriber("/rob2/ur_hardware_interface/io_states", IOStates, self.io_state)
		self.tool_sub = rospy.Subscriber("/tool/control", String, self.tool_cb)
		

		self.run = False
		self.pin = [False,False,False,False] 

		self.table1_state = -0.028
		self.table2_state = -0.165

		self.init_flag = 1
		self.count = 0

		print "set"

	def tool_cb(self, data):
		self.run = False
		if 'tool1' in data.name :

			self.io_pub(1,4,1)
			self.io_pub(1,5,0)

			if data.switch is True:

				if data.speed == 1:
					self.io_pub(1,6,0)
					self.io_pub(1,7,1)

				elif data.speed == 2:
					self.io_pub(1,6,1)
					self.io_pub(1,7,0)

				else:
					self.io_pub(1,6,0)
					self.io_pub(1,7,0)

			else:
				self.io_pub(1,4,1)
				self.io_pub(1,5,1)
				self.io_pub(1,6,1)
				self.io_pub(1,7,1)

		elif 'tool2' in data.name:

			self.io_pub(1, 4, 0)

			self.io_pub(1, 5, 0)

			self.io_pub(1, 6, 1)

			if data.switch is True:

				self.io_pub(1, 7, 1)

			else:
				self.io_pub(1, 7, 0)

		elif 'tool3' in data.name:

			self.io_pub(1,4,0)

			self.io_pub(1,5,1)

			if data.switch is True:

				if data.speed == 1:
					self.io_pub(1,6,0)
					self.io_pub(1,7,1)

				else:
					self.io_pub(1,6,1)
					self.io_pub(1,7,0)
			else:
				self.io_pub(1,4,1)
				self.io_pub(1,5,1)
				self.io_pub(1,6,1)
				self.io_pub(1,7,1)
		else:
			print "no mode"


	def io_state(self, data):
		self.run = data.digital_in_states[1].state
			

	def main(self):
		rospy.spin()
		


if __name__ == '__main__':
	try:
		Tool_pulisher().main()

	except rospy.ROSInterruptException:
	    pass       