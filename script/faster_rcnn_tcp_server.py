#!/usr/bin/env python

# the Tcp Server 
# set up the tcp server and faster-rcnn net, then listen to the client
# receive image path string from client, and response detection result as string
# --------------------------------------------------------

import sys;
#sys.path.append("/home/chenkai/data/frcn/py-faster-rcnn/tools/")
sys.path.append("../py-faster-rcnn/tools/")

import _init_paths
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
from utils.timer import Timer
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
import caffe, os, sys, cv2
import argparse

from SocketServer import TCPServer, BaseRequestHandler
import SocketServer
import traceback
import socket
import thread



CLASSES = ('__background__',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor')

NETS = {'vgg16': ('VGG16',
                  'VGG16_faster_rcnn_final.caffemodel'),
        'zf': ('ZF',
                  'ZF_faster_rcnn_final.caffemodel')}

def demo(net, image_name):

    #im_file = os.path.join(cfg.ROOT_DIR, 'data', 'demo', image_name)
    im_file = image_name
    if os.path.exists(im_file) == False:
	print "file not exist!.."
	return 'file not exist!..' 

    im = cv2.imread(im_file)

    # Detect all object classes and regress object bounds
    timer = Timer()
    timer.tic()
    
    scores, boxes = im_detect(net, im)
    
    timer.toc()
    print ('Detection took {:.3f}s for '
           '{:d} object proposals').format(timer.total_time, boxes.shape[0])

    # Visualize detections for each class
    CONF_THRESH = 0.8
    NMS_THRESH = 0.3
    result = ''
    for cls_ind, cls in enumerate(CLASSES[1:]):
        cls_ind += 1 # because we skipped background
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        dets = np.hstack((cls_boxes, cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, NMS_THRESH)
        dets = dets[keep, :]
        inds = np.where(dets[:, -1] >= CONF_THRESH)[0]

        for i in inds:
            bbox = dets[i, :4]
            score = dets[i, -1]

            result += str(int(bbox[0]))+','+str(int(bbox[1]))+','+str(int(bbox[2]))+','+str(int(bbox[3]))+','+str(cls)+','
   
    #vis_detections(im, cls, dets, thresh=CONF_THRESH)

    return result

def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='Faster R-CNN demo')
    parser.add_argument('--gpu', dest='gpu_id', help='GPU device id to use [0]',
                        default=0, type=int)
    parser.add_argument('--cpu', dest='cpu_mode',
                        help='Use CPU mode (overrides --gpu)',
                        action='store_true')
    parser.add_argument('--net', dest='demo_net', help='Network to use [vgg16]',
                        choices=NETS.keys(), default='vgg16')
    args = parser.parse_args()
    return args

class MyRequestHandler(BaseRequestHandler):

    def handle(self):
        cfg.TEST.HAS_RPN = True  # Use RPN for proposals
        args = parse_args()
        prototxt = os.path.join(cfg.ROOT_DIR, 'models', NETS[args.demo_net][0],
                            'faster_rcnn_alt_opt', 'faster_rcnn_test.pt')
        caffemodel = os.path.join(cfg.ROOT_DIR, 'data', 'faster_rcnn_models',
                              NETS[args.demo_net][1])

        if not os.path.isfile(caffemodel):
            raise IOError(('{:s} not found.\nDid you run ./data/script/'
                       'fetch_faster_rcnn_models.sh?').format(caffemodel))

        if args.cpu_mode:
            caffe.set_mode_cpu()
        else:
            caffe.set_mode_gpu()
            caffe.set_device(args.gpu_id)
            cfg.GPU_ID = args.gpu_id
        net = caffe.Net(prototxt, caffemodel, caffe.TEST)

        print '\n\nLoaded network {:s}'.format(caffemodel)
        # Warmup on a dummy image
        im = 128 * np.ones((300, 500, 3), dtype=np.uint8)
        for i in xrange(2):
            _, _= im_detect(net,im)
        while True:
            try:
                data = self.request.recv(1024).strip()
                print "receive from (%r):%r" % (self.client_address, data)
                if data == 'exit':
		    break
                im_name = data
                result = demo(net, im_name)
                self.request.sendall(result)
            except:
                traceback.print_exc()
                break


def serverLoop(server, info):
    print info
    server.serve_forever()


if __name__ == '__main__':
    
    host = "127.0.0.1"
    port = 9999
    addr = (host, port)

    server = SocketServer.ThreadingTCPServer(addr, MyRequestHandler)
    print 'server start listening to ' + host + ':' + str(port)
    server.serve_forever()  


