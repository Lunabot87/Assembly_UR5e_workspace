
import geometry_msgs.msg
from copy import deepcopy
from tf.transformations import quaternion_from_euler
from math import pi

pose = geometry_msgs.msg.PoseStamped()
pose.header.frame_id = 'world'
pose.pose.orientation.w = 1.0
pose.pose.position.z = 0.81

pin_base11 = deepcopy(pose)
pin_base14 = deepcopy(pose)
pin_base26 = deepcopy(pose)
pin_base29 = deepcopy(pose)

pin_base11.pose.position.x = 0.49
pin_base11.pose.position.y = 0.45
pin_base11.pose.position.z += 0.02

pin_base14.pose.position.x = 0.55
pin_base14.pose.position.y = 0.56
pin_base14.pose.position.z += 0.06

pin_base26.pose.position.x = -0.55
pin_base26.pose.position.y = 0.50
pin_base26.pose.position.z += 0.03

pin_base29.pose.position.x = 0.55
pin_base29.pose.position.y = 0.65
pin_base29.pose.position.z += 0.03

olist = quaternion_from_euler(0,pi,0)

pin_base11.pose.orientation.x =	deepcopy(olist[0]) 
pin_base11.pose.orientation.y = deepcopy(olist[1])
pin_base11.pose.orientation.z = deepcopy(olist[2])
pin_base11.pose.orientation.w = deepcopy(olist[3])

olist = quaternion_from_euler(0,pi/2,0)
pin_base14.pose.orientation.x =	deepcopy(olist[0]) 
pin_base14.pose.orientation.y = deepcopy(olist[1])
pin_base14.pose.orientation.z = deepcopy(olist[2])
pin_base14.pose.orientation.w = deepcopy(olist[3])

pin_base29.pose.orientation.x = deepcopy(olist[0])
pin_base29.pose.orientation.y = deepcopy(olist[1])
pin_base29.pose.orientation.z = deepcopy(olist[2])
pin_base29.pose.orientation.w = deepcopy(olist[3])

olist = quaternion_from_euler(pi/2,0,pi/4)

pin_base26.pose.orientation.x =	deepcopy(olist[0]) 
pin_base26.pose.orientation.y = deepcopy(olist[1])
pin_base26.pose.orientation.z = deepcopy(olist[2])
pin_base26.pose.orientation.w = deepcopy(olist[3])



pin11_TF = {'trans':[0,0,0],'rot':[0,0,0]}
pin14_TF = {'trans':[0,0,0],'rot':[0,pi/2,0]}
pin26_TF = {'trans':[0,-0.01,0.01],'rot':[pi/2,0,0]}
pin29_TF = {'trans':[0,0,0],'rot':[0,pi/2,0]}

pin_TF_pose = [pin11_TF,pin14_TF,pin26_TF,pin29_TF]