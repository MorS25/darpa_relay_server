#!/usr/bin/env python2
import sys
import json
#  from urllib.request import Request, urlopen
#  from urllib.error import HTTPError
from urllib2 import Request, urlopen
from urllib2 import HTTPError
import random
import array

###############################
# test program options

# fake map size
width = 20
height = 20

# compress?
#compress = False
compress = True

# use CBOR?
use_cbor = False
#use_cbor = True

# authentication token
token = 'FKDzKcNTugczqkpC'

###############################
# build our test message

# generate some fake map data
data = array.array('b', [n % 256 - 128 for n in range(width*height)]).tostring()

# build a fake grid message
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
                    u'x' : 1.,
                    u'y' : 2.,
                    u'z' : 3.
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

# optionally compress the data
if compress:
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
     
# put data in correct form depending on content type
if use_cbor:
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
if use_cbor:
    import cbor
    out = cbor.dumps(msg)
else:
    import json
    out = json.dumps(msg).encode('utf-8')
print (str(out))
###############################
# send test message to the HTTP server

url = 'http://127.0.0.1:8000/map/update'
headers = {
        'Content-Type' : 'application/cbor' if use_cbor else 'application/json',
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
    sys.exit(1)

