#!/usr/bin/env python
from __future__ import print_function
import sys
import cbor
import json
import gzip
import base64

import rospy
import tf
from genpy.rostime import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray

# python3 check
_IS_PY3 = sys.version_info[0] >= 3

# http requests library
import requests

# a little recursive type-debugging function
def show(obj, level=0):
    print(type(obj))
    if isinstance(obj, dict):
        for k,v in obj.items():
            print('  '*level, k, ': ', sep='', end='')
            show(v, level+1)
    elif isinstance(obj, list):
        for v in obj:
            print('  '*level, '- ', sep='', end='')
            show(v, level+1)
    elif hasattr(obj, '__slots__'):
        for s in obj.__slots__:
            print('  '*level, s, ': ', sep='', end='')
            show(getattr(obj,s), level+1)
    elif isinstance(obj, tuple):
        print('  '*level, '({})'.format( type(obj[0]) if len(obj) > 0 else ''), sep='')

###############################
# ros to python conversion
###############################

if _IS_PY3:
    def nested(obj):
        if isinstance(obj, Time):
            return obj.to_sec()
        elif hasattr(obj, '__slots__'):
            return dict((s, nested(getattr(obj,s))) for s in obj.__slots__)
        elif isinstance(obj, list):
            return list(map(nested, obj))
        else:
            return obj
else:
    # must convert strings to unicode
    def nested(obj):
        if isinstance(obj, Time):
            return obj.to_sec()
        elif hasattr(obj, '__slots__'):
            d = {}
            for s in obj.__slots__:
                # in python2, we will have already handled the 'data' bytearray to be the correct type
                d[s.decode('utf-8')] = getattr(obj, s) if s is 'data' else nested(getattr(obj, s))
            return d
        elif isinstance(obj, list):
            return map(nested, obj)
        elif isinstance(obj, str):
            return obj.decode('utf-8')
        else:
            return obj

###############################
# gzip compression
###############################

if _IS_PY3:
    def gzip_compress(data):
        return gzip.compress(bytes(data))
else:
    import StringIO
    def gzip_compress(data):
        out = StringIO.StringIO()
        with gzip.GzipFile(fileobj=out, mode='w') as f:
            f.write(str(data))
        return out.getvalue()

###############################
# relay class
###############################

class CommandPostRelay(object):
    def __init__(self):
        if not rospy.has_param('~token'):
            rospy.logfatal("Must specify 'token' parameter.")
            sys.exit(1)
        self.token = rospy.get_param('~token')
        self.map_url = rospy.get_param('~map_url', 'http://10.100.2.201:8000/map/update')
        self.state_url = rospy.get_param('~state_url', 'http://10.100.2.201:8000/state/update')
        self.marker_url = rospy.get_param('~marker_url', 'http://10.100.2.201:8000/markers/update')
        self.compression = rospy.get_param('~compression', 'gzip')

        # HTTP sessions
        self.session = requests.Session()
        self.session.headers['Content-Type']  = 'application/cbor'
        self.session.headers['Authorization'] = 'Bearer ' + self.token
        if self.compression is 'gzip':
            self.session.headers['Content-Encoding'] = 'gzip'
        if self.compression is 'none':
            self.session.headers['Content-Encoding'] = 'identity'

        # TF listener
        self.listener = tf.TransformListener()

        # Occupancy grid subscription
        self.grid_sub  = rospy.Subscriber('grid',  OccupancyGrid, self.handle_grid)
        # Point cloud subscription
        self.cloud_sub = rospy.Subscriber('cloud', PointCloud2,   self.handle_cloud)
        self.pose_sub = rospy.Subscriber('poses', PoseArray, self.handle_poses)
        self.marker_sub = rospy.Subscriber('marker_array', MarkerArray, self.handle_markers)

    def send_map_msg(self, typestr, msg):
        body = cbor.dumps({u'type' : typestr, u'msg' : msg})
        if self.compression is 'gzip':
            body = gzip_compress(body)
            rospy.loginfo('  -> Sending message of size ' + str(len(body)))
        res = self.session.post(self.map_url, body)
        if not res.ok:
            rospy.logerr(str.format('{} {}\n{}', res.status_code, res.reason, res.json()))
    
    def send_plain_msg(self, typestr, msg, url):
        body = cbor.dumps(msg)
        if self.compression is 'gzip':
            body = gzip_compress(body)
        res = self.session.post(url, body)
        if not res.ok:
            rospy.logerr(str.format('{} {}\n{}', res.status_code, res.reason, res.json()))

    def handle_grid(self, msg):
        rospy.loginfo('Relaying grid message with stamp ' + str(msg.header.stamp))
        # For ROS in python2: reintepret tuple of ints as proper byte array
        if not _IS_PY3:
            msg.data = str(bytearray(msg.data))
        # Transform message into a dictionary for easy modification
        msg = nested(msg)
        # Rewrite frame as a test
        msg[u'header'][u'frame_id'] = u'darpa'
        # Send message
        self.send_map_msg(u'OccupancyGrid', msg)

    def handle_cloud(self, msg):
        rospy.loginfo('Relaying cloud message with stamp ' + str(msg.header.stamp))
        # Transform message into a dictionary for easy modification
        msg = nested(msg)
        # Rewrite frame as a test
        msg[u'header'][u'frame_id'] = u'darpa'
        # Send message
        self.send_map_msg(u'PointCloud2', msg)

    def handle_poses(self, msg):
        rospy.loginfo('Relaying poses message with stamp ' + str(msg.header.stamp))
        # Transform message into a dictionary for easy modification
        msg = nested(msg)
        # Rewrite frame as a test
        msg[u'header'][u'frame_id'] = u'darpa'
        # Send message
        self.send_plain_msg(u'PoseArray', msg, self.state_url)

    def handle_markers(self, msg):
        rospy.loginfo('Relaying marker message')
        # Transform message into a dictionary for easy modification
        msg = nested(msg)
        # Send message
        self.send_plain_msg(u'MarkerArray', msg, self.marker_url)


###############################
# run node
###############################

if __name__ == '__main__':
    rospy.init_node('mapping_relay')
    relay = CommandPostRelay()
    rospy.spin()

