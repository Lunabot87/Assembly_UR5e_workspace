
import geometry_msgs.msg
from copy import deepcopy
from tf.transformations import quaternion_from_euler

pose = geometry_msgs.msg.PoseStamped()
pose.header.frame_id = 'world'
pose.pose.orientation.w = 1.0
pose.pose.position.z = 0.81

base11 = deepcopy(pose)
base14 = deepcopy(pose)
base26 = deepcopy(pose)
base29 = deepcopy(pose)

base11.pose.position.x = 0.55
base11.pose.position.y = 0.44

base14.pose.position.x = 0.55
base14.pose.position.y = 0.56
base14.pose.position.z += 0.06

base26.pose.position.x = -0.55
base26.pose.position.y = 0.50
base26.pose.position.z += 0.03

base29.pose.position.x = 0.55
base29.pose.position.y = 0.65
base29.pose.position.z += 0.03

olist = quaternion_from_euler(0,1.57,0)

base14.pose.orientation.x =	deepcopy(olist[0]) 
base14.pose.orientation.y = deepcopy(olist[1])
base14.pose.orientation.z = deepcopy(olist[2])
base14.pose.orientation.w = deepcopy(olist[3])

base29.pose.orientation.x = deepcopy(olist[0])
base29.pose.orientation.y = deepcopy(olist[1])
base29.pose.orientation.z = deepcopy(olist[2])
base29.pose.orientation.w = deepcopy(olist[3])

olist = quaternion_from_euler(1.57,0,0)

base26.pose.orientation.x =	deepcopy(olist[0]) 
base26.pose.orientation.y = deepcopy(olist[1])
base26.pose.orientation.z = deepcopy(olist[2])
base26.pose.orientation.w = deepcopy(olist[3])

pin_pose_base = [base11,base14,base26,base29]



