#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import message_filters
from utils import BaseAlgorithm

class DistanceLidar(BaseAlgorithm):

    def __init__(self, *args, **kargs):
        super().__init__(*args, **kargs)
    
    def detect_crest(self):
        
        th = rospy.get_param('xdiff_threshold')
        cloud = self.lidar_cloud.squeeze()
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

        print(d)

        self.d = d
        
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('distance_lidar', anonymous=True)
    distlidar = DistanceLidar()
    distlidar.run()
    
 

if __name__ == '__main__':
    listener()