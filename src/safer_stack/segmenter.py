import numpy as np
from sklearn import linear_model
from safer_stack.utils import pointcloud2_2_npxyz


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

    def __init__(self, max_angle_dev=30, inter_max_dev=0.80, max_dist=30, n_planes=3, n_jobs=8):
        self.max_angle_dev = max_angle_dev
        self.inter_max_dev = inter_max_dev
        self.max_dist = max_dist
        self.n_planes = n_planes
        self.n_jobs = n_jobs
        
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


# class PlaneObstacleSegmenter:

#     def __init__(self, input_topic='input', output_ground='output_ground', output_obst='output_obstacles'):
#         self.input_topic = input_topic
#         self.output_ground = output_ground
#         self.output_obstacles = output_obst 

#         self.pub_ground = rospy.Publisher(self.output_ground, PointCloud2, queue_size=10)
#         self.pub_obst = rospy.Publisher(self.output_obstacles, PointCloud2, queue_size=10)
#         self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.callback)
    
#     def callback(self, msg):
#         cloud = pointcloud2_2_npxyz(msg)
#         if np.isnan(cloud).sum() > 0:
#             raise ValueError('Cloud should have no Nans')

#         ground, obst = self.compute(cloud.squeeze())
        
#         ground_msg = pc2.create_cloud_xyz32(msg.header, ground)
#         obst_msg = pc2.create_cloud_xyz32(msg.header, obst)

#         self.pub_ground.publish(ground_msg)
#         self.pub_obst.publish(obst_msg)

#     def compute(self, cloud):
        
#         max_angle_dev = 30
#         inter_max_dev = 0.80
#         max_dist = 30
#         n_jobs = 8
#         n_planes = 3

#         remov_points = cloud[~( (cloud[:, 0] < max_dist) & (cloud[:, 1] < max_dist) ), :]
#         cloud = cloud[ (cloud[:, 0] < max_dist) & (cloud[:, 1] < max_dist), :]
#         X = cloud[:, 0:2]
#         Y = cloud[:, -1]

#         Xn = X
#         Yn = Y
#         planes = []
#         lr = linear_model.LinearRegression(n_jobs=n_jobs)
#         ransac = linear_model.RANSACRegressor(lr, residual_threshold=1)
#         for _ in range(n_planes):
#             ransac.fit(Xn, Yn)
#             mask = ransac.inlier_mask_
#             points = np.hstack((Xn[mask, :], Yn[mask].reshape(-1, 1)))
#             coefs = (-ransac.estimator_.coef_[0], -ransac.estimator_.coef_[1], 1)
#             planes.append(
#                 Plane(points,  coefs, -ransac.estimator_.intercept_)
#             )
#             Xn = Xn[~ mask, :]
#             Yn = Yn[~ mask]


#         print(planes)

#         ds = []
#         candidates = []
#         excluded = []
#         for plane in planes:
#             is_valid_ang = plane.angled > (90 - max_angle_dev) and (plane.angled < (90 + max_angle_dev)) 
#             if (np.abs(plane.inter) < inter_max_dev) and is_valid_ang:
#                 candidates.append(plane)
#             else:
#                 excluded.append(plane)

#         print(len(candidates))
#         if len(candidates) > 1:
#             ds = [np.sqrt(np.min(np.sum(p.points**2, axis=-1))) for p in candidates]
#             arg = ds.index(min(ds))
#             ground_plane = candidates[arg]
#             del candidates[arg]
#             excluded.extend(candidates)
#         elif len(candidates) == 1:
#             ground_plane = candidates[0]
                
#         obst_points = np.vstack([p.points for p in excluded] + [remov_points])
#         # obst_points = np.vstack((obst_points, ))
#         return ground_plane.points, obst_points

if __name__ == '__main__':

    pass