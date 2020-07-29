#!/usr/bin/env python
from UR5_ulol import*
import ur5_inv_kin_wrapper as IKW

def rearrange_solution(ik_solution):
    new_sol = IKW.np.zeros((8,6))
    for i in range(0,8):
        for j in range(0,6):
            new_sol[i][j] = ik_solution[j][i]
    return new_sol

def make_transform_matrix(pose):
  orientation_list = make_orientation_list(pose.orientation)
  T = quaternion_matrix(orientation_list)
  T[0][3] = pose.position.x 
  T[1][3] = pose.position.y
  T[2][3] = pose.position.z
  return T

def make_orientation_list(orientation):
  o_list = [orientation.x, orientation.y, orientation.z, orientation.w]
  return o_list


def main():
  ROB1 = UR5()
  ur5_inv = IKW.ur5_inv_kin_wrapper()
  listener = tf.TransformListener()
  print "============ INITIALIZE DONE"

  while not rospy.is_shutdown():
    try:
      print "============ Press `Enter`  to get_current_joint_values"
      raw_input()
      cur_joint = ROB1.move_group.get_current_joint_values()
      (trans, rot) = listener.lookupTransform('/rob1_real_base_link', '/rob1_real_ee_link', rospy.Time(0))
      ur5_inv.solve_and_sort(trans, rot, cur_joint)
      A = []
      s = []
      j = 0
      ur5_inv._print_sol(ur5_inv.inv_sol)
      for i in range(8):
        rv = ur5_inv._get_state_validity(ur5_inv.inv_sol[:,i])
        if rv == True:
          j += 1
          s.append(i)
          A.append(ur5_inv.inv_sol[:,i])          

      ur5_inv.inv_sol = A
      raw_input()
      print "="*17+"VALID  SOLUTION"+"="*17
      for i in range(j):
        print ""
        print "PRESS ENTER"
        raw_input()
        print 'sol #',s[i]
        ur5_inv.publish_state(i)
      print('='*50)
      print "complete!" 

    except rospy.ROSInterruptException:
      return
    except KeyboardInterrupt:
      return



if __name__ == '__main__':
    main()