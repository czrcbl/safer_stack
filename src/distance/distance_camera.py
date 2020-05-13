#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2

from utils import BaseAlgorithm


class DistanceCamera(BaseAlgorithm):

    def __init__(self, *args, **kargs):
        super().__init__(*args, **kargs)

    def algorithm0(self, cloud, th):
        # works on transformed cloud
        th = 1
        # x_nnan, y_nnan = np.nonzero(~np.isnan(cloud[:, :, 0]))
        # points = cloud[x_nnan, y_nnan, :]
        points = cloud.squeeze()

        angles = np.arctan2(points[:, 1], points[:, 0])
        ang_mask = np.logical_and(angles < np.pi / 12, angles > -np.pi/12)
        fpoints = points[ang_mask, :]
        args = np.argsort(fpoints[:, 0])
        sorted_x = fpoints[args, 0]
        diff = sorted_x[1:] - sorted_x[:-1]
        mask = diff > th
        idxs = np.nonzero(mask)[0]
        # print(idxs)
        # 124752
        # print(sorted_x[idxs[0] - 10:idxs[0]+10])

        if len(idxs) > 0:
            d = sorted_x[idxs[0]]
        else:
            d = sorted_x[-1]
        # if len(np.nonzero(mask)) > 0:
        #     p  = sorted_x[np.nonzero(mask)]
        #     if len(p) > 1:
        #         p = p[0]
        #     p = p.flatten()
        #     d = p
        #     # d = np.sqrt(np.sum(p**2))
        # else:
        #     d = np.float('nan')

        return d

    def algorithm1(self, cloud, th):
        # Works on transformed cloud
        window_size = (9, 9)
        bin_width = 0.5

        x_nnan, y_nnan = np.nonzero(~np.isnan(cloud[:, :, 0]))
        points = cloud[x_nnan, y_nnan, :]

        angles = np.arctan2(points[:, 1], points[:, 0])
        ang_mask = np.logical_and(angles < np.pi / 12, angles > -np.pi/12)
        fpoints = points[ang_mask, :]
        args = np.argsort(fpoints[:, 0])
        sorted_x = fpoints[args, 0]

        low = np.min(fpoints[:, 0])
        high = np.max(fpoints[:, 0])

        bins = np.arange(low, high, bin_width)

        digitized = np.digitize(sorted_x, bins)
        bin_counts = []
        for i in range(1, digitized.max() + 1):
            bin_counts.append(np.sum(digitized == i))

        mask = (np.array(bin_counts) == 0)
        idxs = np.nonzero(mask)[0]
        d = bins[idxs[0]] + bin_width / 2
        return d 

    def algorithm3(self, cloud, th):

        height_threshold = 0.5
        th = 1
        # x_nnan, y_nnan = np.nonzero(~np.isnan(cloud[:, :, 0]))
        # points = cloud[x_nnan, y_nnan, :]
        points = cloud.squeeze()
        angles = np.arctan2(points[:, 1], points[:, 0])
        ang_mask = np.logical_and(angles < np.pi / 6, angles > -np.pi/6)
        fpoints = points[ang_mask, :]
        # print(fpoints[:, -1])
        # height_mask = np.logical_and(fpoints[:, -1] < height_threshold, fpoints[:, -1] > -height_threshold)
        # fpoints = fpoints[height_mask, :]

        args = np.argsort(fpoints[:, 0])
        sorted_x = fpoints[args, 0]
        sorted_z = fpoints[args, -1]
        sorted_points = fpoints[args, :]
        print(sorted_points[-1, :])
        diffx = sorted_x[1:] - sorted_x[:-1]
        diffz = np.abs(sorted_z[1:] - sorted_z[:-1])
        # mask = np.logical_and(diffx > th, diffz > height_threshold)
        mask = np.abs(sorted_z) > 1
        # mask = np.logical_or(diffz > height_threshold,  np.abs(sorted_z[:-1]) > 1)
        idxs = np.nonzero(mask)[0]
        if len(idxs) > 0:
            d = sorted_x[idxs[0]]
        else:
            d = sorted_x[-1]
        
        return d

    def detect_crest(self):
        
        if self.camera_cloud is None: return
        cloud = self.camera_cloud
        th = rospy.get_param('xdiff_threshold')

        d = self.algorithm3(cloud, th)

        print(d)
        self.d = d


def listener():
    
    rospy.init_node('distance_camera', anonymous=True)

    # distcamera = DistanceCamera(camera_cloud_topic='/bumblebee2/point_cloud_transformed')
    distcamera = DistanceCamera(camera_cloud_topic='/rtabmap/cloud_ground_transformed')
    distcamera.run()
    

if __name__ == '__main__':
    listener()