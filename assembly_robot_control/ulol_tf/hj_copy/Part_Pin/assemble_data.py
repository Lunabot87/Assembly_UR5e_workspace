import rospy
from copy import deepcopy

from part_info import*			# part_file_address, part_name

class Assemble_Data(object):
	def __init__(self):
		super(Assemble_Data, self).__init__()
		self.a_list = {'part':[],'pin':[]}
		self.init_ASM_list()

	def init_ASM_list(self):
		self.ASM_List = [{'part':[],'pin':[]},{'part':[],'pin':[]},{'part':[],'pin':[]},
							{'part':[],'pin':[]},{'part':[],'pin':[]},{'part':[],'pin':[]}]

	def print_ASM_list(self, ASMLIST):
		for i in range(6):
			print "\n+++++++++++++"
			print "At PART #",i+1
			linked_part = len(ASMLIST[i]['part'])
			
			print "part=========="
			for part in range(linked_part):
				print ASMLIST[i]['part'][part],"\t",
			print""
			print "pin==========="
			linked_pin = len(ASMLIST[i]['pin'])
			for pin in range(linked_pin):
				print ASMLIST[i]['pin'][pin],"\t",
				if pin%7 == 6:
					print ""

	def assemble_part(self,target_part_name,assembling_part_name): # both names have to be string type
		if target_part_name in part_name and assembling_part_name in part_name:
			target_part_num = part_name.index(target_part_name)
			assembling_part_num = part_name.index(assembling_part_name)

			self.ASM_List[target_part_num]['part'].append(deepcopy(assembling_part_name))
			self.ASM_List[assembling_part_num]['part'].append(deepcopy(target_part_name))
		else:
			print "assemble_part : [WRONG_ATTACH_LIST]"

	def assemble_pin(self,target_part_name,assembling_pin_name): # both names have to be string type
		target_part_num = part_name.index(target_part_name)
		assembling_pin_num = pin_name.index(assembling_pin_name.split('-')[0])
		# assembling_pin_tag = assembling_pin_name.split('-')[1]-1
		self.ASM_List[target_part_num]['pin'].append(deepcopy(assembling_pin_name))
		self.ASM_List[target_part_num]['pin'].sort()

	def init_attach_list(self):
		self.a_list = {'part':[],'pin':[]}

	def get_attach_list(self,target_mesh_name):
		attach_list = {'part':[],'pin':[]}
		attached_stack = []		# have been appended
		attaching_queue = []	# have to be appended
		attaching_queue.append(deepcopy(target_mesh_name))

		if target_mesh_name.split('-')[0] in pin_name:
			attach_list['pin'].append(deepcopy(target_mesh_name))

		elif target_mesh_name in part_name:

			while not attaching_queue == []:				
				target_part_num = part_name.index(attaching_queue[0])

				linked_part_list = self.ASM_List[target_part_num]['part']

				for linked_part_name in linked_part_list:
					if not linked_part_name in attached_stack:
						attaching_queue.append(deepcopy(linked_part_name))

				attach_list['part'].append(deepcopy(attaching_queue[0]))
				attached_stack.append(deepcopy(attaching_queue[0]))
				del attaching_queue[0]

				linked_pin_list = self.ASM_List[target_part_num]['pin']

				for target_pin_name in linked_pin_list:
					if not target_pin_name in attach_list['pin']:
						attach_list['pin'].append(deepcopy(target_pin_name))
			self.a_list = attach_list
			
		else:
			print "(get_attach_list) : WRONG_MESH_NAME"
			
		return attach_list



# def main():
# 	try:
# 		ASM = Assemlbe_Data()
		
# 		ASM.assemble_pin(part_name[4],pin_name[0]+"-"+str(2))
# 		ASM.assemble_pin(part_name[4],pin_name[0]+"-"+str(3))
# 		ASM.assemble_pin(part_name[4],pin_name[0]+"-"+str(1))
# 		ASM.assemble_pin(part_name[4],pin_name[0]+"-"+str(21))
# 		ASM.assemble_part(part_name[4],part_name[1])
# 		ASM.assemble_part(part_name[4],part_name[2])
# 		ASM.assemble_pin(part_name[1],pin_name[0]+"-"+str(4))
# 		ASM.assemble_pin(part_name[1],pin_name[0]+"-"+str(5))
# 		ASM.print_ASM_list(self.ASM_List)

# 		L = ASM.get_attach_list(part_name[4])
# 		print L



# 	except rospy.ROSInterruptException:
# 		return
# 	except KeyboardInterrupt:
# 		return

# if __name__ == '__main__':
# 	main()
# 	