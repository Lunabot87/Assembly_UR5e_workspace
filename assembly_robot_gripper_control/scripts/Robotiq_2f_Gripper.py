import socket
import time

class Robotiq_2f_Gripper(object):

	PORT = 30002


	def __init__(self, robot_ip):
		rob_socket =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		rob_socket.connect((robot_ip, 30002))
		time.sleep(0.05)

		self.socket = rob_socket

		self._set_gripper()
		time.sleep(2)
		self.action_gripper(0)



	def _set_gripper(self, speed = 255, force = 100):

		socket = self.socket

		msg  = "def UR5_gripper_set():\n"
		msg += "\tsocket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
		#set input and output voltage ranges
		msg += "\tset_analog_inputrange(0,0)\n"
		msg += "\tset_analog_inputrange(1,0)\n"
		msg += "\tset_analog_inputrange(2,0)\n"
		msg += "\tset_analog_inputrange(3,0)\n"
		msg += "\tset_analog_outputdomain(0,0)\n"
		msg += "\tset_analog_outputdomain(1,0)\n"
		msg += "\tset_tool_voltage(0)\n"
		msg += "\tset_runstate_outputs([])\n"
		#set payload, speed and force
		msg += "\tset_payload(1.1)\n"
		msg += "\tsocket_set_var(\"SPE\",%d,\"gripper_socket\")\n"%(speed)
		msg += "\tsync()\n"
		msg += "\tsocket_set_var(\"FOR\",%d,\"gripper_socket\")\n"%(force)
		msg += "\tsync()\n"
		#initialize the gripper
		msg += "\tsocket_set_var(\"ACT\", 1, \"gripper_socket\")\n"
		msg += "\tsync()\n"
		msg += "\tsocket_set_var(\"GTO\", 1, \"gripper_socket\")\n"
		msg += "\tsync()\n"
		msg += "\tsocket_set_var(\"POS\", 0, \"gripper_socket\")\n"
		msg += "\tsync()\n"
		msg += "\tsleep(0.5)\n"
		msg += "\ttextmsg(\"gripper setting complete\")\n"
		msg += "end\n"
		socket.send(msg.encode('ascii'))
		time.sleep(2)
		print("gripper_set")


	def action_gripper(self, action_pos, wait_time = 2):
		socket = self.socket

		msg  = "def UR5_gripper_action():\n"
		msg += "\tsocket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
		msg += "\tsocket_set_var(\"POS\", %s, \"gripper_socket\")\n"%(str(action_pos)) #catched chair!
		msg += "end\n"
		socket.send(msg.encode('ascii'))
		time.sleep(wait_time)

if __name__ == '__main__':
	grip = Robotiq_2f_Gripper("192.168.13.101")
