#!/usr/bin/env python

from UR5_ulol import*
from Parts_ulol import*

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print ""
    ROB1 = UR5()
    ROB2 = UR5('rob2_arm')
    Ch = Parts('chair_part1')
    print "[[[[[[[initialize DONE]]]]]]]]]]"
    print "============ Press `Enter` ...2"
    raw_input()
    ROB1.go_to_initial_pose()
    ROB2.go_to_initial_pose()
    Ch.add_mesh(part2)
    mesh_pose = Ch.get_mesh_pose('chair_part1').pose
    position_list = [mesh_pose.position.x, mesh_pose.position.y, mesh_pose.position.z]
    orientation_list = make_orientation_list(mesh_pose.orientation)
    print "[[[[[[[mesh add done]]]]]]"
    
    print "============ Press `Enter` ...3"
    raw_input()
    mesh_pose.orientation = ROB1.get_target_orientation(mesh_pose.orientation,Ch.xyz_length)

    ROB1.grab_part(target_name = 'chair_part1', target_file = part2, target_pose = mesh_pose, target_size = Ch.xyz_length, offset = 0.2)
    ROB1.detach_part('chair_part1')
    print "============ complete!"
    print "============ PRESS ENTER TO END"
    raw_input()
  
  except rospy.ROSInterruptException:
  	return
  except KeyboardInterrupt:
  	return
if __name__ == '__main__':
 main()