#!/usr/bin/env python3
from detectron2 import model_zoo
import _fix
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import cv2
from safer_stack3.nn import Segmenter
from safer_stack.utils import pointcloud2_2_npxyz


class ObstacleNN:

    def __init__(self):

        self.predictor = Segmenter()

        self.color_img = None
        self.rect_img = None
        self.cloud = None

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

    def rect_img_callback(self, data):
        self.rect_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def lidar_points2_callback(self, data):

        self.lidar_cloud = pointcloud2_2_npxyz(data)

    def publish(self):
        
        if self.color_img is None: return

        im = self.color_img
        dim = self.predictor.pred_and_draw(im)
        cv2.imshow('Segmentation', dim)
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
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # rospy.loginfo('Message sent!')
        obsnn.publish()
        rate.sleep()

if __name__ == '__main__':
    listener()