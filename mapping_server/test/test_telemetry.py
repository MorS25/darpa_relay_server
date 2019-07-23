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

# use CBOR?
use_cbor = False
#use_cbor = True

# authentication token
token = 'FKDzKcNTugczqkpC'

###############################
# build our test message

msg = {
        u'header' : {
            u'stamp' : 1234.0,
            u'frame_id' : u'darpa'
            },
        u'poses' : [
            {
                u'name' : u'alpha',
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
                },
            {
                u'name' : u'bravo',
                u'position' : {
                    u'x' : 4.,
                    u'y' : 5.,
                    u'z' : 6.
                    },
                u'orientation' : {
                    u'x' : 0.,
                    u'y' : 0.,
                    u'z' : 0.,
                    u'w' : 1.
                    }
                },
            {
                u'name' : u'gamma',
                u'position' : {
                    u'x' : 7.,
                    u'y' : 8.,
                    u'z' : 9.
                    },
                u'orientation' : {
                    u'x' : 0.,
                    u'y' : 0.,
                    u'z' : 0.,
                    u'w' : 1.
                    }
                }

            ]
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

url = 'http://127.0.0.1:8000/state/update'
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

