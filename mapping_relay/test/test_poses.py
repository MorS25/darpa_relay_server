#!/user/bin/env python

import rospy
import random
import sys

from geometry_msgs.msg import Pose, PoseStamped

def test_poses(num_robots=3):
        publishers = []
	for i in range(num_robots):
                publishers.append(rospy.Publisher('poses/robot'+str(i+1), PoseStamped, queue_size=10))
	rospy.init_node('test_poses')
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		posemsg = PoseStamped()
		posemsg.header.stamp = rospy.Time.now()
		posemsg.header.frame_id = 'darpa'
		for i in range(num_robots):
			pose = Pose()
			pose.position.x = random.random() * 10
			pose.position.y = random.random() * 10
			pose.position.z = random.random() * 10
			pose.orientation.w = 1
			posemsg.pose = pose
		        publishers[i].publish(posemsg)
                rate.sleep()

if __name__ == '__main__':
  num_robots = 3
  if len(sys.argv) > 1:
    num_robots = sys.argv[1]
  try:
    test_poses(int(num_robots))
  except rospy.ROSInterruptException:
    pass

