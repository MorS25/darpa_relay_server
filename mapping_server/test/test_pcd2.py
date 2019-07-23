#!/usr/bin/env python2
import sys
import json
#  from urllib.request import Request, urlopen
#  from urllib.error import HTTPError
from urllib2 import Request, urlopen
from urllib2 import HTTPError
import random
import array
import struct

###############################
# test program options

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

numpts = random.randint(30, 100)
floats = []
for i in range(numpts*3):
   floats.append(random.uniform(-10,10))
data = struct.pack("%sf"%len(floats), *floats)

# build a fake grid message
cloud = {
        u'header' : {
            u'stamp' : 1234.0,
            u'frame_id' : u'darpa'
            },
        u'height' : 1,
        u'width' : numpts,
        u'fields' : [
                {
                    u'name': 'x', 
                    u'offset': 0,
                    u'datatype': 7,
                    u'count': 1},
                {
                    u'name': 'y', 
                    u'offset': 4,
                    u'datatype': 7,
                    u'count': 1},
                {
                    u'name': 'z', 
                    u'offset': 8,
                    u'datatype': 7,
                    u'count': 1},
                ],
        u'is_bigendian': False,
        u'point_step': 12,
        u'row_step': 12 * numpts,
        u'is_dense': False
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
    cloud[u'compression'] = u'gzip'
else:
    compressed = data
     
# put data in correct form depending on content type
if use_cbor:
    cloud[u'data'] = compressed
else:
    import base64
    cloud[u'data'] = base64.b64encode(compressed)

# create the test message
msg = {
        u'type' : u'PointCloud2',
        u'msg' : cloud
        }

# serialize the test message
if use_cbor:
    import cbor
    out = cbor.dumps(msg)
else:
    import json
    out = json.dumps(msg).encode('utf-8')
###############################
# send test message to the HTTP server
print(str(out))
url = 'http://127.0.0.1:8000/map/update'
headers = {
        'Content-Type' : 'application/cbor' if use_cbor else 'application/json',
        'Authorization' : 'Bearer ' + token
        }
print(str(headers))
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


