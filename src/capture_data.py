#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time
import argparse
import sys
from utils import pointcloud2_2_npxyz, project_path


class DataCapture:

    def __init__(self, camera_topic, cloud_topic, output_prefix, pub_rate=5):
        
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.img_callback)
        self.cloud_sub = rospy.Subscriber(cloud_topic, PointCloud2, self.cloud_callback)
        
        self.pub_rate = pub_rate
        self.bridge = CvBridge()
        self.out_pref = output_prefix
        self.i = 0

        self.img = None
        self.cloud = None


    def img_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.img = img


    def cloud_callback(self, msg):
        self.cloud = pointcloud2_2_npxyz(msg)

    def save(self):

        if self.img is None or self.cloud is None: return

        np.save(self.out_pref + '/cloud_{}'.format(self.i),self.cloud)
        cv2.imwrite(self.out_pref + '/img_{}.jpg'.format(self.i), self.img)
        print('Example {}.'.format(self.i))
        self.i += 1

    def run(self):

        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            rate.sleep()
            self.save()
        
def main():

    argv = rospy.myargv(argv=sys.argv)

    parser = argparse.ArgumentParser(description='Capture camere and lidar data.')
    parser.add_argument('--folder', type=str, default='{}_experimente'.format(time.time()),
                        help='Subfolder to save experiments')

    args = parser.parse_args(argv[1:])

    output_prefix = project_path + '/data/{}'.format(args.folder)
    if not os.path.isdir(output_prefix):
        os.makedirs(output_prefix)
    rospy.init_node('capture_data', anonymous=True)
    
    dc = DataCapture(
        camera_topic='/bumblebee2/left/image_rect_color',
        cloud_topic='/bumblebee2/point_cloud_transformed',
        output_prefix=output_prefix
    )
    
    dc.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass