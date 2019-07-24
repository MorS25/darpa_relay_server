#!/user/bin/env python

import rospy
import random

from geometry_msgs.msg import PoseArray, Pose

def test_telem():
	publisher = rospy.Publisher('poses', PoseArray, queue_size=10)
	rospy.init_node('test_telem')
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		posemsg = PoseArray()
		posemsg.header.stamp = rospy.Time.now()
		posemsg.header.frame_id = 'darpa'
		for i in range(3):
			pose = Pose()
			pose.position.x = random.random() * 10
			pose.position.y = random.random() * 10
			pose.position.z = random.random() * 10
			pose.orientation.w = 1
			posemsg.poses.append(pose)

		publisher.publish(posemsg)
		rate.sleep()

if __name__ == '__main__':
	try:
		test_telem()
	except rospy.ROSInterruptException:
		pass

