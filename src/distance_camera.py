#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2


class DistanceCloud:

    def __init__(self):

        self.raw_img = None
        self.rect_img = None
        self.cloud = None

        self.d = None
        self.crest = None

        self.bridge = CvBridge()
        self.pub_dist = rospy.Publisher('/edge_distance', Float32, queue_size=10)
        self.pub_img = rospy.Publisher('/image_edge_highlighted', Image, queue_size=10)
        
        rospy.Subscriber('/bumblebee2/left/image_raw', Image, self.raw_img_callback)
        rospy.Subscriber('/bumblebee2/left/image_rect', Image, self.rect_img_callback)
        rospy.Subscriber('/bumblebee2/points2', PointCloud2, self.points2_callback)

    def raw_img_callback(self, data):
        self.raw_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def rect_img_callback(self, data):
        self.rect_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def points2_callback(self, data):
        h = data.height
        w = data.width
    
        coords = np.zeros(shape=(h, w, 3))
        for n, point in enumerate(pc2.read_points(data, skip_nans=False)):
            # print(point[0]) 
            j = n % w
            i = n // w
            coords[i, j, :] = point[:3]
        
        self.cloud = coords

        self.detect_crest()

    def detect_crest(self):
        th = 20
        # fill_val = 100
        cloud = self.cloud
        # cloudc = np.array(cloud)
        # cloudc = cloudc[~np.isnan(cloud)] 
        # print(cloudc.shape)
        x_nnan, y_nnan = np.nonzero(~np.isnan(cloud[:, :, 0]))
        points = cloud[x_nnan, y_nnan, :]
        args = np.argsort(points[:, -1])
        sorted_x = points[args, -1]
        diff = sorted_x[1:] - sorted_x[:-1]
        mask = diff > th
        if len(np.nonzero(mask)) > 0:
            p  = sorted_x[np.nonzero(mask)]
            if len(p) > 1:
                p = p[0]
            p = p.flatten()
            d = np.sqrt(np.sum(p**2))
        else:
            d = np.float('nan')

        print(d)

    def draw_crest(self):
        if self.crest is None: return
        raw_img = np.array(self.raw_img)
        raw_img[self.crest, np.arange(raw_img.shape[1]), :] = np.array([255, 0, 0])
        self.raw_img_draw = raw_img
        return self.raw_img
        

    def publish(self):
        
        # if (self.d is None) or (self.crest is None): return

        # raw_img_draw = self.draw_crest()
        # out_img_msg = self.bridge.cv2_to_imgmsg(raw_img_draw)
        # self.pub_img.publish(out_img_msg)

        dmsg = Float32()
        dmsg.data = self.d
        self.pub_dist.publish(dmsg)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('DistanceCloud', anonymous=True)

    distcloud = DistanceCloud()
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # rospy.loginfo('Message sent!')
        distcloud.publish()
        rate.sleep()

if __name__ == '__main__':
    listener()