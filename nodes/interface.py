#!/usr/bin/env python
import rospy 
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
# from safer_stack.utils import create_message


class Interface:

    def __init__(self, edge_distance_topic='/edge_distance'):
        self.edge_distance_topic = edge_distance_topic

        rospy.Subscriber(edge_distance_topic, Float32, self.distance_callback)
        rospy.Subscriber('/crest', numpy_msg(Float32), self.crest_callback)
        rospy.Subscriber('/stereo_camera/left/image_raw', Image, self.image_callback)
        
        self.bridge = CvBridge()
        self.image = None
        self.d = None
        self.crest = None

    def distance_callback(self, data):
        self.d = data.data

    def crest_callback(self, data):
        self.crest = data.data

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def draw_image(self):
        
        image = np.array(self.image)
        d = self.d
        color = (255,0,0)
        font_scale = 2
        cv2.putText(image, 'Distance: {:.2f}'.format(d), (100, 150), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color)
        
        return image

    def update_interface(self):
        if (self.image is None) or (self.d is None):
            return
        # np_image = np.asarray(cv_image)
        image = self.draw_image()
        cv2.imshow('Rear Camera - {}'.format(self.edge_distance_topic), cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
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