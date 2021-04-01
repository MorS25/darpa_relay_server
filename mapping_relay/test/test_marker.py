#!/user/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

def test_marker():
	publisher = rospy.Publisher('marker', Marker, queue_size=10)
	rospy.init_node('test_marker')
	rate = rospy.Rate(1)
	#team = rospy.get_param('team','DARPA')
	while not rospy.is_shutdown():
		markermsg = Marker(
			type=Marker.TEXT_VIEW_FACING,
			id=0,
			lifetime=rospy.Duration(),
			pose=Pose(Point(0,0,0), Quaternion(0,0,0,1)),
			scale=Vector3(1, 1, 1),
			header=Header(frame_id='darpa'),
			color=ColorRGBA(1.0,1.0,1.0,1.0),
			text='DARPA')
		publisher.publish(markermsg)
		rate.sleep()

if __name__ == '__main__':
	try:
		test_marker()
	except rospy.ROSInterruptException:
		pass

