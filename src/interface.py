#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from utils import create_message


class Interface:

    def __init__(self):
        rospy.Subscriber('/edge_distance', Float32, self.distance_callback)
        rospy.Subscriber('/image_edge_highlighted', Image, self.image_callback)
        self.bridge = CvBridge()
        self.image = None
        self.d = None

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def distance_callback(self, data):
        self.d = data.data

    def update_interface(self):
        if (self.image is None) or (self.d is None):
            return
        # np_image = np.asarray(cv_image)
        image = self.image
        d = self.d
        color = (0,0,255)
        font_scale = 2
        cv2.putText(image, 'Distance: {:.2f}'.format(d), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color)
        cv2.imshow('Rear Camera', cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        cv2.waitKey(3)


def main():

    rospy.init_node('Interface', anonymous=True)
    rate = rospy.Rate(10)
    
    inter = Interface()

    while not rospy.is_shutdown():
        inter.update_interface()
        rate.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass