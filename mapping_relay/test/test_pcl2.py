#!/user/bin/env python

import rospy
import random

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def test_cloud():
	publisher = rospy.Publisher('slam/cloud', PointCloud2, queue_size=10)
	rospy.init_node('test_pcl2')
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'darpa'

		fields = [
			PointField('x', 0, PointField.FLOAT32, 1),
			PointField('y', 4, PointField.FLOAT32, 1),
			PointField('z', 8, PointField.FLOAT32, 1)]

		points = []
		for i in range(1000):
			point = [random.random() * 10, random.random() * 10, random.random()*10]
			points.append(point)

		cloudmsg = point_cloud2.create_cloud(header, fields, points)

		publisher.publish(cloudmsg)
		rate.sleep()

if __name__ == '__main__':
	try:
		test_cloud()
	except rospy.ROSInterruptException:
		pass

