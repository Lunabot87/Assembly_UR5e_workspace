import rospy
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
pub_marker = rospy.Publisher('marker_test', Marker, queue_size=10)

def pub_sphere(trans = [0,0,0.81],id_num = 0):
	SPM = Marker()
	SPM.header.frame_id = 'world'
	SPM.ns = "Spheres" # unique ID
	SPM.type = Marker().SPHERE

	SPM.id = id_num

	

	SPM.pose.position.x = trans[0]
	SPM.pose.position.y = trans[1]
	SPM.pose.position.z = trans[2]
	SPM.pose.orientation.w = 1.0

	SPM.color.r = 1.0
	SPM.color.g = 0.0
	SPM.color.b = 0.0
	SPM.color.a = 1.0

	SPM.lifetime = rospy.Duration(90.0)

	pub_marker.publish(SPM)

def pub_sphere_list(pose_list):
	SPM = Marker()
	SPM.header.frame_id = 'world'
	SPM.ns = "Spheres" # unique ID
	SPM.type = Marker().SPHERE_LIST


	SPM.lifetime = rospy.Duration(30.0)

	SPM.header.stamp = rospy.Time.now()

	SPM.scale.x = 0.0008
	SPM.scale.y = 0.0008
	SPM.scale.z = 0.0008

	SPM.color.r = 1.0
	SPM.color.g = 0.0
	SPM.color.b = 0.0
	SPM.color.a = 1.0

	spheres_color = ColorRGBA()
	spheres_color.r = 1.0
	spheres_color.g = 0.0
	spheres_color.b = 0.0
	spheres_color.a = 1.0



	SPM.points[:] = []
	SPM.colors[:] = []
	for point in pose_list:
		pose = geometry_msgs.msg.PoseStamped().pose.position
		pose.x = point[0]
		pose.y = point[1]
		pose.z = point[2]
		SPM.points.append(pose) 
		SPM.colors.append(spheres_color)

	pub_marker.publish(SPM)