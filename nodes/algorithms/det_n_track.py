#!/usr/bin/env python
from __future__ import print_function, division
import sys
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import cv2
import time
import mxnet as mx

import matplotlib.pyplot as plt
from safer_stack.utils import pointcloud2_2_npxyz, compute_distances
from safer_stack.nn import Segmenter, Detector
from safer_stack.tracking import MultiTracker


class ObstacleNN:

    def __init__(self, model_type='segmentation', frames_per_det=30):
        
        self.model_type = model_type

        if model_type == 'detection':
            self.predictor = Detector(ctx=mx.gpu())
        elif model_type == 'segmentation':
            self.predictor = Segmenter(ctx=mx.gpu())

        # NN
        self.th = 0.7
        self.predictions = None
        self.first_trigg = True

        self.tracker = MultiTracker()
        self.iter = 0

        self.frames_per_det = frames_per_det # number of frames between new CNN predictions are added.

        self.color_img = None
        self.rect_img = None
        self.camera_cloud = None
        self.deltas = []
        self.tp = None

        self.d = None
        self.crest = None

        self.bridge = CvBridge()
        self.pub_dist = rospy.Publisher('/edge_distance', Float32, queue_size=10)
        self.pub_img = rospy.Publisher('/image_edge_highlighted', Image, queue_size=10)
        
        rospy.Subscriber('/stereo_camera/left/image_rect_color', Image, self.color_img_callback)
        rospy.Subscriber('/stereo_camera/left/image_rect', Image, self.rect_img_callback)
        rospy.Subscriber('/stereo_camera/points2', PointCloud2, self.points2_callback)

        
    def color_img_callback(self, data):
        self.color_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        # self.color_img is RGB
        
        if self.first_trigg:
            self.apply_network()
            self.first_trigg = False
        
        self.publish()

    def rect_img_callback(self, data):
        self.rect_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def points2_callback(self, data):

        self.camera_cloud = pointcloud2_2_npxyz(data)

    def fps(self):
        t = time.time()
        if self.tp is None:
            self.tp = t
            return 0
        
        d = t - self.tp
        self.deltas.append(d)
        self.deltas = self.deltas[-10:]
        fps = 1/np.mean(self.deltas)
        self.tp = t
        return fps


    # def compute_distances(self, bboxlist, npim):

    #     cloud = self.camera_cloud
    #     dists = []
    #     rbboxes = [bbox.resize(npim.shape[:2], cloud.shape[:2]) for bbox in bboxlist]
    #     for rb in rbboxes:
    #         class_name = rb.class_name
    #         c = rb.crop_image(cloud)
    #         c = c.reshape((-1, 3))
    #         idx = ~np.isnan(c[:, 0]) 
    #         c = c[idx, :]
    #         d = np.mean(c, axis=0)
    #         d = np.sqrt(np.sum(d ** 2))
    #         dists.append((class_name, d))

    #     return dists

    def apply_network(self, event=None):
        im = self.color_img
        if im is None: return
        th = self.th
        tic = time.time()

        if self.model_type == 'detection':
            bboxlist, npim = self.predictor.detect(im, th=th, keep_size=True)
            
        elif self.model_type == 'segmentation':
            seglist, bboxlist, npim = self.predictor.detect(im, th=th, keep_size=True)

        print('Model Execution Time:', time.time() - tic)
        
        bboxlist = bboxlist.remove_overlap()

        self.predictions = bboxlist



    def publish(self):
        
        if self.color_img is None: return
        if self.camera_cloud is None: return

        # im = self.color_img[:, :, ::-1] # Convert to BGR
        im = self.color_img # Keep RGB
        
        # plt.imshow(im)
        # plt.show()
        
        cloud = self.camera_cloud
        th = 0.7
        
        ##############################

        # if (self.iter % self.frames_per_det) == 0:

        #     tic = time.time()

        #     if self.model_type == 'detection':
        #         bboxlist, npim = self.predictor.detect(im, th=th, keep_size=True)
                
        #     elif self.model_type == 'segmentation':
        #         seglist, bboxlist, npim = self.predictor.detect(im, th=th, keep_size=True)

        #     print('Model Execution Time:', time.time() - tic)
            
        #     bboxlist = bboxlist.remove_overlap()
            
        #     bboxlist = self.tracker.update(im, bboxlist)

        # else:
        #     bboxlist = self.tracker.update(im)
        
        # self.iter += 1

        ##########################3

        # if self.model_type == 'detection':
        #     bboxlist, npim = self.predictor.detect(im, th=th, keep_size=True)
            
        # elif self.model_type == 'segmentation':
        #     seglist, bboxlist, npim = self.predictor.detect(im, th=th, keep_size=True)

        ############################


        
        bboxlist = self.tracker.update(im, bboxlist=self.predictions)
        self.predictions = None
       
        dists = compute_distances(cloud, bboxlist, im.shape[:2])
        print(dists)
        dim = bboxlist.draw(im)
        # tbboxes = self.tracker.track(bboxlist)
        # dim = tbboxes.draw(npim)
    

        fps = self.fps()
        print('FPS:', fps)

        cv2.imshow('Segmentation', dim[:,:,::-1])
        cv2.waitKey(3)


        # raw_img_draw = self.draw_crest()
        # out_img_msg = self.bridge.cv2_to_imgmsg(raw_img_draw)
        # self.pub_img.publish(out_img_msg)

        # dmsg = Float32()
        # dmsg.data = self.d
        # self.pub_dist.publish(dmsg)

    
def main():

    rospy.init_node('DistanceCloud', anonymous=True)

    obsnn = ObstacleNN()
    
    rospy.Timer(rospy.Duration(secs=5), obsnn.apply_network)

    # rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # rospy.loginfo('Message sent!')
        # obsnn.publish()
        rospy.spin()
        # rate.sleep()

if __name__ == '__main__':
    main()