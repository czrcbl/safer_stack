#!/usr/bin/env python
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
from safer_stack.utils import pointcloud2_2_npxyz
from safer_stack.nn import Segmenter, Detector
from safer_stack.tracking import Tracker


class ObstacleNN:

    def __init__(self, model_type='detection'):
        
        self.model_type = model_type

        if model_type == 'detection':
            self.predictor = Detector()
        elif model_type == 'segmentation':
            self.predictor = Segmenter()

        self.tracker = Tracker()

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


    def compute_distances(self, bboxlist, npim):

        cloud = self.camera_cloud
        dists = []
        rbboxes = [bbox.resize(npim.shape[:2], cloud.shape[:2]) for bbox in bboxlist]
        for rb in rbboxes:
            class_name = rb.class_name
            c = rb.crop_image(cloud)
            c = c.reshape((-1, 3))
            idx = ~np.isnan(c[:, 0]) 
            c = c[idx, :]
            d = np.mean(c, axis=0)
            d = np.sqrt(np.sum(d ** 2))
            dists.append((class_name, d))

        return dists

    def publish(self):
        
        if self.color_img is None: return
        if self.camera_cloud is None: return

        im = self.color_img[:, :, ::-1]
        cloud = self.camera_cloud
        
        
        if self.model_type == 'detection':
            bboxlist, npim = self.predictor.detect(im)
            dists = self.compute_distances(bboxlist, npim)
        elif self.model_type == 'segmentation':
            seglist, bboxlist, npim = self.predictor.detect(im)
            dists = self.compute_distances(bboxlist, npim)
        
        tbboxes = self.tracker.track(bboxlist)
        dim = tbboxes.draw(npim)
       
        print(dists)


        fps = self.fps()
        print('FPS:', fps)

        cv2.imshow('Segmentation', dim[:,:,:])
        cv2.waitKey(3)


        # raw_img_draw = self.draw_crest()
        # out_img_msg = self.bridge.cv2_to_imgmsg(raw_img_draw)
        # self.pub_img.publish(out_img_msg)

        # dmsg = Float32()
        # dmsg.data = self.d
        # self.pub_dist.publish(dmsg)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('DistanceCloud', anonymous=True)

    obsnn = ObstacleNN()
    
    # rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # rospy.loginfo('Message sent!')
        # obsnn.publish()
        rospy.spin()
        # rate.sleep()

if __name__ == '__main__':
    listener()