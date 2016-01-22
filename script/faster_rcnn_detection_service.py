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

import os

global client
server_temp_path = "/home/chenkai/tmp/0000.jpg"
host = "166.111.82.128"
port = 9999
user_name = "chenkai"

def handle_rcnn_demo(req):
    debug_info = "Returning Detection Results for %s" % (req.path)
    print debug_info

    # method 1, copy file and send message    
    #scp_transfer_file(req.path, server_temp_path, host, user_name)
    #client.send(server_temp_path)
    
    # method 2, just send message
    client.send(req.path)
    data = client.recv(512)
    if len(data) > 0:
        result = data
    
    # result = '100,100,300,300,person'
    print "tcp server response:" + result 
    return FasterRcnnDetectionResponse(debug_info, result)

def run_faster_rcnn_detection_service():
    rospy.init_node('faster_rcnn_detection')
    s = rospy.Service('faster_rcnn_detection', FasterRcnnDetection, handle_rcnn_demo)
    print "ready to object detection"
    rospy.spin()

def scp_transfer_file(local_path, target_path, host, user_name):
    command = "scp -p "
    command += local_path
    command += " "
    target_path = server_temp_path
    command += user_name + "@" + host + ":" + target_path
    
    os.system(command)
    


if __name__ == '__main__':
   
    addr = (host, port)

    # copy the file


    # setup the tcp connection
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
    
    run_faster_rcnn_detection_service()

    client.close()

