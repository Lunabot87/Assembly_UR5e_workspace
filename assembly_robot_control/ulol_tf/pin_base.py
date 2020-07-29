
import geometry_msgs.msg
from copy import deepcopy

pose = geometry_msgs.msg.PoseStamped()
pose.pose.orientation.w = 1.0
pose.pose.position.z = 0.81

base11 = deepcopy(pose)
base14 = deepcopy(pose)
base26 = deepcopy(pose)
base29 = deepcopy(pose)

base11.pose.position.x = -0.55
base11.pose.position.y = 0.3

base14.pose.position.x = 0.55
base14.pose.position.y = 0.3

base26.pose.position.x = -0.55
base26.pose.position.y = -0.3

base29.pose.position.x = 0.55
base29.pose.position.y = -0.3

pin_pose_base = [base11,base14,base26,base29]
pin_side = [-1,-1,1,1]		# which side is the pin located. rob1 = -1 rob2 = 1
