#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import pickle
import time
from utils import pointcloud2_2_npxyz, project_path

output_path = project_path + '/data/{}_sim_data.pkl'.format(time.time())

bridge = CvBridge()

data = {
    'images': [],
    'clouds': []
}
i = 0
def callback(image, cloud):
    global data, i
    xyz = pointcloud2_2_npxyz(cloud)
    img = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
    data['images'].append(img)
    data['clouds'].append(xyz)
    if i == 15:
        with open(output_path, 'wb') as f:
            pickle.dump(data, f)
    i += 1
    print(i)

def main():

    rospy.init_node('capture_data')
    image_sub = message_filters.Subscriber('/bumblebee2/left/image_rect_color', Image)
    cloud_sub = message_filters.Subscriber('/bumblebee2/points2', PointCloud2)

    ts = message_filters.TimeSynchronizer([image_sub, cloud_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass