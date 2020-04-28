#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
# import struct

from sklearn import linear_model
import sensor_msgs.point_cloud2 as pc2
from utils import pointcloud2_2_npxyz


# def callback(msg):

#     cloud = pointcloud2_2_npxyz(msg)
#     if np.isnan(cloud).sum() > 0:
#         raise ValueError('Cloud should have no Nans')
    
#     cloud = cloud.squeeze()
#     ground, obst = ground_segmentation(cloud)
    
#     ground_msg = pc2.create_cloud_xyz32(msg.header, ground)
#     obst_msg = pc2.create_cloud_xyz32(msg.header, obst)

#     pub_ground.publish(ground_msg)
#     pub_obst.publish(obst_msg)

# def ground_segmentation(cloud, iter_cycle = 10, threshold = 0.40):
#     xyz = cloud
#     print(xyz.shape)
#     # height_col = int(np.argmin(np.var(xyz, axis = 0)))
#     height_col = 2

#     # Make a new column for index 
#     temp = np.zeros((len(xyz[:,1]),4), dtype= float)
#     temp[:,:3] = xyz[:,:3]
#     temp[:,3] = np.arange(len(xyz[:,1]))
#     xyz = temp

#     # Filter the points based on the height value
#     z_filter = xyz[(xyz[:,height_col]< np.mean(xyz[:,height_col]) + 1.5*np.std(xyz[:,height_col])) & (xyz[:,height_col]> np.mean(xyz[:,height_col]) - 1.5*np.std(xyz[:,height_col]))]

#     # Normalize as per the height filter value
#     max_z, min_z = np.max(z_filter[:,height_col]), np.min(z_filter[:,height_col])
#     z_filter[:,height_col] = (z_filter[:,height_col] - min_z)/(max_z - min_z) 

#     print(z_filter.shape)
    
#     for i in range(iter_cycle):
#         print(z_filter.shape)
#         covariance = np.cov(z_filter[:,:3].T)
#         w,v,h = np.linalg.svd(np.matrix(covariance))
#         normal_vector = w[np.argmin(v)]
        
#         #Resample points
#         filter_mask = np.asarray(np.abs(np.matrix(normal_vector)*np.matrix(z_filter[:,:3]).T )<threshold)
#         # print(filter_mask.shape)
#         print(np.sum(filter_mask))
#         z_filter = z_filter[filter_mask.squeeze(), :]
#         # z_filter = np.asarray([z_filter[index[1]] for index,a in np.ndenumerate(filter_mask) if a == True])

#     z_filter[:,height_col] = z_filter[:,height_col]*(max_z - min_z) + min_z
#     world = np.array([row for row in xyz if row[3] not in z_filter[:,3]])


#     ground = z_filter[:,:3]

#     return ground, world

# if __name__ == '__main__':
	
#     rospy.init_node('ground_segment', anonymous = True)

#     pub_ground = rospy.Publisher('/bumblebee2/ground', PointCloud2, queue_size=10)
#     pub_obst = rospy.Publisher('/bumblebee2/obstacles', PointCloud2, queue_size=10)
#     sub = rospy.Subscriber("/bumblebee2/points2_filtered_transformed", PointCloud2, callback)
        
#     rate = rospy.Rate(10)

#     while not rospy.is_shutdown():
#         rate.sleep()

class Plane:
    
    def __init__(self, points, coefs, inter):
        self.points = points
        self.coefs =  coefs
        self.inter = inter
        
    def __repr__(self):
        return 'Plane angled={}, inter={}'.format(self.angled, self.inter)
        
    @property
    def angle(self):
        a, b, c = self.coefs
        x = np.sqrt(a**2 + b**2)
        ang = np.arctan2(c, x)
        
        return ang
    
    @property
    def angled(self):
        
        return self.angle * 180 / np.pi


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

        ground, obst = self.compute(cloud.squeeze())
        
        ground_msg = pc2.create_cloud_xyz32(msg.header, ground)
        obst_msg = pc2.create_cloud_xyz32(msg.header, obst)

        self.pub_ground.publish(ground_msg)
        self.pub_obst.publish(obst_msg)

    def compute(self, cloud):
        
        max_angle_dev = 30
        inter_max_dev = 0.80
        max_dist = 30
        n_jobs = 8
        n_planes = 3

        remov_points = cloud[~( (cloud[:, 0] < max_dist) & (cloud[:, 1] < max_dist) ), :]
        cloud = cloud[ (cloud[:, 0] < max_dist) & (cloud[:, 1] < max_dist), :]
        X = cloud[:, 0:2]
        Y = cloud[:, -1]

        Xn = X
        Yn = Y
        planes = []
        lr = linear_model.LinearRegression(n_jobs=n_jobs)
        ransac = linear_model.RANSACRegressor(lr, residual_threshold=1)
        for _ in range(n_planes):
            ransac.fit(Xn, Yn)
            mask = ransac.inlier_mask_
            points = np.hstack((Xn[mask, :], Yn[mask].reshape(-1, 1)))
            coefs = (-ransac.estimator_.coef_[0], -ransac.estimator_.coef_[1], 1)
            planes.append(
                Plane(points,  coefs, -ransac.estimator_.intercept_)
            )
            Xn = Xn[~ mask, :]
            Yn = Yn[~ mask]


        print(planes)

        ds = []
        candidates = []
        excluded = []
        for plane in planes:
            is_valid_ang = plane.angled > (90 - max_angle_dev) and (plane.angled < (90 + max_angle_dev)) 
            if (np.abs(plane.inter) < inter_max_dev) and is_valid_ang:
                candidates.append(plane)
            else:
                excluded.append(plane)

        print(len(candidates))
        if len(candidates) > 1:
            ds = [np.sqrt(np.min(np.sum(p.points**2, axis=-1))) for p in candidates]
            arg = ds.index(min(ds))
            ground_plane = candidates[arg]
            del candidates[arg]
            excluded.extend(candidates)
        elif len(candidates) == 1:
            ground_plane = candidates[0]
                
        obst_points = np.vstack([p.points for p in excluded] + [remov_points])
        # obst_points = np.vstack((obst_points, ))
        return ground_plane.points, obst_points


if __name__ == '__main__':

    rospy.init_node('cloud_seg', anonymous=True)

    seg = PlaneObstacleSegmenter(
        input_topic='/bumblebee2/points2_filtered_transformed',
        output_ground='/bumblebee2/ground',
        output_obst='/bumblebee2/obstacles'
    )
    rospy.spin()