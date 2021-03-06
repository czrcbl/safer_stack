from __future__ import print_function, division
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import quaternion_from_euler
from rospy.numpy_msg import numpy_msg
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError


def create_twist_message(linear, angular=[0,0,0]):
    """
    Create a twist message for the husky
    """
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


def create_modelstate_message(coords, yaw, model_name='/'):
    """
    Create a model state message for husky robot
    """
    pose = Pose()
    p = Point()
    p.x = coords[0]
    p.y = coords[1]
    p.z = coords[2]
    pose.position = p
    
    qua = quaternion_from_euler(0, 0, yaw)
    q = Quaternion()
    q.x = qua[0]
    q.y = qua[1]
    q.z = qua[2]
    q.w = qua[3]
    pose.orientation = q
    
    twist = Twist()
    twist.linear = Vector3(0, 0, 0)
    twist.angular = Vector3(0, 0, 0)

    ms = ModelState()

    ms.model_name = model_name
    ms.pose = pose
    ms.twist = twist
    ms.reference_frame = 'sand_mine'

    return ms


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


class BaseAlgorithm(object):

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


def compute_distances(cloud, bboxlist, border=0.0, metric='min'):

    dists = []
    # rbboxes = [bbox.resize(bbox_img_size, cloud.shape[:2]) for bbox in bboxlist]
    for bb in bboxlist:
        class_name = bb.class_name
        c = bb.crop_image(cloud, border=border)
        c = c.reshape((-1, 3))
        idx = ~np.isnan(c[:, 0]) 
        c = c[idx, :]
        if metric=='min':
            ds = np.sqrt(np.sum(c ** 2, axis=1))
            d = np.min(ds)
        elif metric=='mean':
            d = np.mean(c, axis=0)
            d = np.sqrt(np.sum(d ** 2))
        
        dists.append((class_name, d))

    return dists
    