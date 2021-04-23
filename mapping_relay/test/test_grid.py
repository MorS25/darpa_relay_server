#!/user/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import array
import random
import sys

def test_grid(topic):
	publisher = rospy.Publisher(topic, OccupancyGrid, queue_size=10)
	rospy.init_node('test_grid', anonymous=True)
	rate = rospy.Rate(0.5)
	while not rospy.is_shutdown():
		gridmsg = OccupancyGrid()
		gridmsg.header.stamp = rospy.Time.now()
		gridmsg.header.frame_id = 'darpa'
		gridmsg.info.resolution = 0.5
		gridmsg.info.width = 100
		gridmsg.info.height = 100
		gridmsg.info.origin.orientation.w = 1
		gridmsg.data = array.array('i')
		for y in range(gridmsg.info.height):
			for x in range(gridmsg.info.width):
				gridmsg.data.append(random.randint(0,100))
		publisher.publish(gridmsg)
		rate.sleep()

if __name__ == '__main__':
  topic = 'slam/grid'
  if len(sys.argv) > 1:
    topic = sys.argv[1]
  try:
    test_grid(topic)
  except rospy.ROSInterruptException:
    pass
