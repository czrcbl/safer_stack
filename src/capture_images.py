#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time
from utils import project_path

output_path = project_path

bridge = CvBridge()

i = 0
def callback(image, cloud):
    global i
    rate = 20
    img = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if i % rate == 0:
        cv2.imwrite(project_path + '/data/images/{}.jpg'.format(i//rate), img)
        print(i)
    i += 1

def main():

    rospy.init_node('capture_image', anonymous=True)
    image_sub = rospy.Subscriber('/bumblebee2/left/image_rect_color', Image, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass