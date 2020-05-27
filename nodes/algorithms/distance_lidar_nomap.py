#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from rospy.numpy_msg import numpy_msg

from safer_stack.utils import pointcloud2_2_npxyz

class DistanceLidar:

    def __init__(self, edge_topic='/lidar/edge_distance',
        crest_topic='/lidar/crest',
        lidar_cloud_topic='/lidar/points2_transformed'):
    
        self.pub_dist = rospy.Publisher(edge_topic, Float32, queue_size=10)
        self.pub_crest = rospy.Publisher(crest_topic, numpy_msg(Float32), queue_size=10)
        
        rospy.Subscriber(lidar_cloud_topic, PointCloud2, self.lidar_points2_callback)

    def lidar_points2_callback(self, data):

        self.lidar_cloud = pointcloud2_2_npxyz(data)
        self.publish()
    
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

    def publish(self):
        
        self.detect_crest()
        dmsg = Float32()
        dmsg.data = self.d
        self.pub_dist.publish(dmsg)
        
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('distance_lidar', anonymous=True)
    distlidar = DistanceLidar(
        edge_topic='/lidar/edge_distance',
        crest_topic='/lidar/crest'
    )
    while True:
        rospy.spin()
    
 

if __name__ == '__main__':
    listener()
