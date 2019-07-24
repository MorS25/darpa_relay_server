#!/usr/bin/env python2
import sys
import json
#  from urllib.request import Request, urlopen
#  from urllib.error import HTTPError
from urllib2 import Request, urlopen
from urllib2 import HTTPError
import random
import array
import rospy
from nav_msgs.msg import OccupancyGrid
import time
import argparse
import math

token = 'DarpaTestToken00'

def make_request(args, out, url_suffix):
    ###############################
    # send test message to the HTTP server

    url = 'http://'+str(args.server)+':8000'+url_suffix
    headers = {
        'Content-Type' : 'application/cbor' if args.use_cbor else 'application/json',
        'Authorization' : 'Bearer ' + token
        }
    req = Request(url=url, headers=headers, data=out)
    try:
        res = urlopen(req)
        print(res.msg)
        print(res.info())
        print(json.loads(res.read().decode('utf-8')))
    except HTTPError as err:
        print(err)
        print(err.info())
        print(json.loads(err.read().decode('utf-8')))
def gen_rand_quat():
    x = random.uniform(-1,1)
    y = random.uniform(-1,1)
    z = random.uniform(-1,1)
    w = random.uniform(-1,1)
    tot = math.sqrt(x*x + y*y + z*z + w*w)
    x = x / tot
    y = y / tot
    z = z / tot
    w = w / tot
    return {'x':x, 'y':y, 'z':z, 'w':w}
def send_poses(args):
    pose_num = random.randint(1, 50)
    poses = []
    for i in range(pose_num):
        randquat = gen_rand_quat()
        poses.append({
                u'name' : u'pose'+str(i),
                u'position' : {
                    u'x' : random.uniform(-20, 20),
                    u'y' : random.uniform(-20, 20),
                    u'z' : random.uniform(-10, 10)
                    },
                u'orientation' : {
                    u'x' : randquat['x'],
                    u'y' : randquat['y'],
                    u'z' : randquat['z'],
                    u'w' : randquat['w']
                    }
                })
    msg = {
        u'header' : {
            u'stamp' : 1234.0,
            u'frame_id' : u'darpa'
            },
        u'poses' : poses
        }
    # serialize the test message
    if args.use_cbor:
        import cbor
        out = cbor.dumps(msg)
    else:
        import json
        out = json.dumps(msg).encode('utf-8')

    make_request(args, out, '/state/update')

def send_markers(args):
    pass
def send_ptcld(args):
    pass
def send_map(args):
    width = random.randint(20, 2001)
    height = random.randint(20, 2001)
    origin_x = random.uniform(-10, 10)
    origin_y = random.uniform(-10, 10)
    grid = {
        u'header' : {
            u'stamp' : 1234.0,
            u'frame_id' : u'darpa'
            },
        u'info' : {
            u'resolution' : 0.05,
            u'width' : width,
            u'height' : height,
            u'origin' : {
                u'position' : {
                    u'x' : origin_x,
                    u'y' : origin_y,
                    u'z' : 0.
                    },
                u'orientation' : {
                    u'x' : 0.,
                    u'y' : 0.,
                    u'z' : 0.,
                    u'w' : 1.
                    }
                }
            }
        }
    data = array.array('b', [n % 100 for n in range(width*height)]).tostring()
    # optionally compress the data
    if args.compress:
        import gzip
        import StringIO
        gzout = StringIO.StringIO()
        with gzip.GzipFile(fileobj=gzout, mode='w') as f:
            f.write(data)
        compressed = gzout.getvalue()
        #  data = gzip.compress(data)
        grid[u'compression'] = u'gzip'
    else:
        compressed = data
#    print(str(data))
    # put data in correct form depending on content type
    if args.use_cbor:
        grid[u'data'] = compressed
    else:
        import base64
        grid[u'data'] = base64.b64encode(compressed)

    # create the test message
    msg = {
        u'type' : u'OccupancyGrid',
        u'msg' : grid
        }
    # serialize the test message
    if args.use_cbor:
        import cbor
        out = cbor.dumps(msg)
    else:
        import json
        out = json.dumps(msg).encode('utf-8')
    make_request(args, out, '/map/update')



def main():
    parser = argparse.ArgumentParser(description='Regression test client for mapping server')
    parser.add_argument('--compress', action='store_true')
    parser.add_argument('--use_cbor', action='store_true')
    parser.add_argument('--server', help='Server address', default='10.100.2.201')
    parser.add_argument('--num-maps', help='Number of maps to send', type=int, default=0)
    parser.add_argument('--num-telemetry', help='Number of telemetry messages to send', type=int, default=0)
    parser.add_argument('--num-interleaved', help='Number of messages to interleave all options', type=int, default=0)
    args = parser.parse_args()
    print args
    for i in range(args.num_maps):
        send_map(args)
        time.sleep(0.1)
    for i in range(args.num_telemetry):
        send_poses(args)
        time.sleep(0.1)
    for i in range(args.num_interleaved):
        send_map(args)
        send_poses(args)
        time.sleep(0.1)

if __name__ == "__main__":
    main()
