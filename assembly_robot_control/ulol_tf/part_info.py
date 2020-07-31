part_address = "/home/kdh/assembly_ws/src/Assembly_UR5e_workspace/assembly_robot_control/ulol_tf/object_description/chair_meshes/"
part1 = part_address+"chair part1.SLDPRT.STL"
part2 = part_address+"chair part2.SLDPRT.STL"
part3 = part_address+"chair part3.SLDPRT.STL"
part4 = part_address+"chair part4.SLDPRT.STL"
part5 = part_address+"chair part5.SLDPRT.STL"
part6 = part_address+"chair part6.SLDPRT.STL"
pin11 = part_address + "101350.SLDPRT.STL"
pin14 = part_address + "104322.SLDPRT.STL"
pin26 = part_address + "122620.SLDPRT.STL"
pin29 = part_address + "122925.SLDPRT.STL"



part_file = [part1,part2,part3,part4,part5,part6]
pin_file = [pin11,pin14,pin26,pin29]
part_name = ["chair_part1","chair_part2","chair_part3","chair_part4","chair_part5","chair_part6"]
pin_name = ["pin101350","pin104322","pin122620","pin122925"]

pin_diameter = [0.008,0.006,0.01,0.006]
pin_lenght = [0.01,0.02,0.01,0.005]
pin_has_hole = [False,False,True,False]
how_many_pins = [21,6,4,4]
# pin_diameter[2] and pin_lenght[2] have to be corrected