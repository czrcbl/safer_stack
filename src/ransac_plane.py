#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sklearn import linear_model
from utils import pointcloud2_2_npxyz


class PlaneObstacleSegmenter:

    def __init__(self, input_topic='input', output_ground='output_ground', output_obst='output_obstacles'):
        self.input_topic = input_topic
        self.output_ground = output_ground
        self.output_obstacles = output_obst 

        self.pub_ground = rospy.Publisher(self.output_ground, PointCloud2, queue_size=10)
        self.pub_obst = rospy.Publisher(self.output_obstacles, PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.callback)
    
    def callback(self, msg):
        cloud = pointcloud2_2_npxyz(msg)
        if np.isnan(cloud).sum() > 0:
            raise ValueError('Cloud should have no Nans')
        ground, obst = self.compute(cloud)
        
        ground_msg = pc2.create_cloud_xyz32(msg.header, ground)
        obst_msg = pc2.create_cloud_xyz32(msg.header, obst)

        self.pub_ground.publish(ground_msg)
        self.pub_obst.publish(obst_msg)

    def compute(self, cloud):
        
        n_planes = 5
        X = cloud[0, :, 0:2]
        Y = cloud[0, :, -1]

        Xn = X
        Yn = Y
        planes = []
        ransac = linear_model.RANSACRegressor(linear_model.LinearRegression())
        for _ in range(n_planes):
            ransac.fit(Xn, Yn)
            mask = ransac.inlier_mask_
            planes.append(np.hstack((Xn[mask, :], Yn[mask].reshape(-1, 1))))
            Xn = Xn[~ mask, :]
            Yn = Yn[~ mask]

        
        ds = []
        for plane in planes:
            min_d = np.sqrt(np.min(np.sum(plane**2, axis=-1)))
            ds.append(min_d)

        arg = ds.index(min(ds))
        ground = planes[arg]
        del planes[arg]
        obst = np.vstack(planes)
        
        return ground, obst


if __name__ == '__main__':

    rospy.init_node('cloud_seg', anonymous=True)

    seg = PlaneObstacleSegmenter(
        input_topic='/bumblebee2/point_cloud_transformed',
        output_ground='/bumblebee2/ground',
        output_obst='/bumblebee2/obstacles'
    )
    rospy.spin()