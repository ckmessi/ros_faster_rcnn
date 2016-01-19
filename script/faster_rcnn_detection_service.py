#!/usr/bin/env python

# --------------------------------------------------------
# Faster R-CNN
# Copyright (c) 2015 Microsoft
# Licensed under The MIT License [see LICENSE for details]
# Written by Ross Girshick
# --------------------------------------------------------

"""
Demo script showing detections in sample images.

See README.md for installation instructions before running.
"""
import sys;
import roslib; roslib.load_manifest('ros_faster_rcnn')
from ros_faster_rcnn.srv import *
import rospy
import time
from SocketServer import TCPServer, BaseRequestHandler
import SocketServer
import traceback
import socket
import thread


global client

def handle_rcnn_demo(req):
    debug_info = "Returning Detection Results for %s" % (req.path)
    print debug_info
    #client.send(req.path)
    #data = client.recv(512)
    #if len(data) > 0:
    #    result = data
    result = '100,100,300,300,person'
    return FasterRcnnDetectionResponse(debug_info, result)

def run_faster_rcnn_detection_service():
    rospy.init_node('faster_rcnn_detection')
    s = rospy.Service('faster_rcnn_detection', FasterRcnnDetection, handle_rcnn_demo)
    print "ready to object detection"
    rospy.spin()

if __name__ == '__main__':
   
    ''' 
    host = "127.0.0.1"
    port = 9999
    addr = (host, port)

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            client.connect(addr)
            print "client connect..."
            break
        except Exception, e:
            time.sleep(0.1)
            print "connect failed.."
            continue
    '''
    run_faster_rcnn_detection_service()

    # client.close()

