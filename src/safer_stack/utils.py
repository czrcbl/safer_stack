from __future__ import print_function, division
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from rospy.numpy_msg import numpy_msg
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError

project_path = os.path.abspath(os.path.realpath(os.path.dirname(os.path.dirname(__file__))))


def create_message(linear, angular=[0,0,0]):
    
    linearm = Vector3()
    linearm.x = linear[0]
    linearm.y = linear[1]
    linearm.z = linear[2]
    angularm = Vector3()
    angularm.x = angular[0]
    angularm.y = angular[1]
    angularm.z = angular[2]

    twist = Twist()
    twist.linear = linearm
    twist.angular = angularm

    return twist


def pointcloud2_2_npxyz(data):
    h = data.height
    w = data.width

    coords = np.zeros(shape=(h, w, 3))
    for n, point in enumerate(pc2.read_points(data, skip_nans=False)):
        # print(point[0]) 
        j = n % w
        i = n // w
        coords[i, j, :] = point[:3]
    
    return coords


class BaseAlgorithm:

    def __init__(self, pub_rate=5, 
        edge_topic='/edge_distance',
        crest_topic='/crest', color_img_topic='/camera/', 
        image_topic='/bumblebee2/left/image_rect',
        image_color_topic='/bumblebee2/left/image_rect_color',
        lidar_cloud_topic='/lidar/points2_transformed',
        camera_cloud_topic='/bumblebee2/point_cloud_transformed'):

        self.pub_rate = pub_rate
        self.gray_img = None
        self.color_img = None
        self.lidar_cloud = None
        self.camera_cloud = None
        
        self.d = np.float('nan')
        self.crest = np.zeros(shape=(1,), dtype=np.float)

        self.bridge = CvBridge()
        self.pub_dist = rospy.Publisher(edge_topic, Float32, queue_size=10)
        self.pub_crest = rospy.Publisher(crest_topic, numpy_msg(Float32), queue_size=10)
        
        rospy.Subscriber(image_topic, Image, self.img_callback)
        rospy.Subscriber(color_img_topic, Image, self.color_img_callback)
        rospy.Subscriber(lidar_cloud_topic, PointCloud2, self.lidar_points2_callback)
        rospy.Subscriber(camera_cloud_topic, PointCloud2, self.camera_points2_callback)

    def img_callback(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def color_img_callback(self, data):
        self.color_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def lidar_points2_callback(self, data):

        self.lidar_cloud = pointcloud2_2_npxyz(data)

    def camera_points2_callback(self, data):

        self.camera_cloud = pointcloud2_2_npxyz(data)

    def detect_crest(self):
        pass

    def publish(self):
        
        self.detect_crest()
        self.pub_crest.publish(self.crest)

        dmsg = Float32()
        dmsg.data = self.d
        self.pub_dist.publish(dmsg)

    def run(self):

        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            rate.sleep()
            self.publish()
    