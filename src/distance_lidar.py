#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import message_filters


class DistanceCloud:

    def __init__(self):


        self.cloud = None

        self.d = None
        self.crest = None

        self.pub_dist = rospy.Publisher('/edge_distance', Float32, queue_size=10)
        self.pub_img = rospy.Publisher('/image_edge_highlighted', Image, queue_size=10)
        
        rospy.Subscriber('/velodyne/point_cloud_transformed', PointCloud2, self.points2_callback)

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

    # def detect_crest(self):
        
    #     th = 10

    #     cloud = self.cloud
    #     # sorted_x_cloud = np.sort(cloud, axis=0)
    #     xs = cloud[0, :, 0]
    #     sorted_x = np.sort(xs)
    #     diff = sorted_x[1:] - sorted_x[:-1]
    #     mask = diff > th
    #     if len(np.nonzero(mask)) > 0:
    #         p  = sorted_x[np.nonzero(mask)]
    #         if len(p) > 1:
    #             p = p[0]
    #         p = p.flatten()
    #         d = np.sqrt(np.sum(p**2))
    #     else:
    #         d = np.float('nan')

        
    #     # print(sorted_x_cloud.shape)
    #     # p = sorted_x_cloud[0, -1, :]

    #     print(d)

    #     self.d = d

    def detect_crest(self):
        
        th = rospy.get_param('xdiff_threshold')

        cloud = self.cloud.squeeze()
        # print(cloud.shape)
        # print(cloud[:, 1], cloud[:, 0])
        angles = np.arctan2(cloud[:, 1], cloud[:, 0])
        # print(np.max(angles))
        # print(angles)
        ang_mask = np.logical_and(angles < np.pi / 6, angles > -np.pi/6)
        # print(ang_mask.sum())
        fcloud = cloud[ang_mask, :]
        xs = fcloud[:, 0]
        sorted_x = np.sort(xs)
        diff = sorted_x[1:] - sorted_x[:-1]
        mask = diff > th
        i_idx = np.nonzero(mask)[0]
        if len(i_idx) > 0:
            p  = sorted_x[i_idx]
            if len(p) > 1:
                p = p[0]
            p = p.flatten()
            d = np.sqrt(np.sum(p**2))
        else:
            d = np.max(sorted_x)
        
        # print(sorted_x_cloud.shape)
        # p = sorted_x_cloud[0, -1, :]

        print(d)

        self.d = d

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