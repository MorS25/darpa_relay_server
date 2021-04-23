#!/user/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

def test_markers():
	publisher = rospy.Publisher('markers', MarkerArray, queue_size=10)
	rospy.init_node('test_markers')
	rate = rospy.Rate(1)
	#team = rospy.get_param('team','DARPA')
	while not rospy.is_shutdown():
		markermsg = MarkerArray()
                darpa_marker = Marker(
			type=Marker.TEXT_VIEW_FACING,
			id=0,
			lifetime=rospy.Duration(1),
			pose=Pose(Point(0,0,2), Quaternion(0,0,0,1)),
			scale=Vector3(1, 1, 1),
			header=Header(frame_id='darpa'),
			color=ColorRGBA(1.0,1.0,1.0,1.0),
			text='DARPA')
                subt_marker = Marker(
			type=Marker.TEXT_VIEW_FACING,
			id=1,
			lifetime=rospy.Duration(1),
			pose=Pose(Point(0,0,0), Quaternion(0,0,0,1)),
			scale=Vector3(1, 1, 1),
			header=Header(frame_id='darpa'),
			color=ColorRGBA(1.0,1.0,1.0,1.0),
			text='SubT Challenge')
                markermsg.markers.append(darpa_marker)
                markermsg.markers.append(subt_marker)
		publisher.publish(markermsg)
		rate.sleep()

if __name__ == '__main__':
	try:
		test_markers()
	except rospy.ROSInterruptException:
		pass

