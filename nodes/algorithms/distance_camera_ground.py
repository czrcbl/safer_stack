#!/usr/bin/env python
# ROS imports
import rospy
from std_msgs.msg import Float32, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from rospy.numpy_msg import numpy_msg
from tf.transformations import euler_from_quaternion, quaternion_from_matrix

# Utils
import sys

# Math
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler
from sklearn import linear_model

# Safer Stack
from safer_stack.utils import pointcloud2_2_npxyz
from safer_stack.segmenter import Plane, PlaneObstacleSegmenter


class CameraDistOdom:

    def __init__(self, pub_rate=5, 
        edge_topic='/edge_distance',
        crest_topic='/crest', color_img_topic='/camera/', 
        image_topic='/bumblebee2/left/image_rect',
        image_color_topic='/bumblebee2/left/image_rect_color',
        lidar_cloud_topic='/lidar/point_cloud_transformed',
        camera_cloud_topic='/bumblebee2/point_cloud_transformed',
        odom_topic='/odom',
        crest_cloud_topic='/rtabmap/crest_cloud',
        alg_num=0):

        self.alg_num = alg_num
        self.pub_rate = pub_rate
        self.gray_img = None
        self.color_img = None
        self.lidar_cloud = None
        self.camera_cloud = None

        self.position = None
        self.orientation = None
        self.R = None
        
        self.seq = 0

        self.d = np.float('nan')
        self.crest = None
        self.crest_cloud = None

        self.bridge = CvBridge()
        self.pub_dist = rospy.Publisher(edge_topic, Float32, queue_size=10)
        self.pub_crest = rospy.Publisher(crest_topic, numpy_msg(Float32), queue_size=10)
        self.pub_crest_cloud = rospy.Publisher(crest_cloud_topic, PointCloud2, queue_size=10)
        
        rospy.Subscriber(image_topic, Image, self.img_callback)
        rospy.Subscriber(color_img_topic, Image, self.color_img_callback)
        rospy.Subscriber(lidar_cloud_topic, PointCloud2, self.lidar_points2_callback)
        rospy.Subscriber(camera_cloud_topic, PointCloud2, self.camera_points2_callback)

        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

    def img_callback(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def color_img_callback(self, data):
        self.color_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def lidar_points2_callback(self, data):
        self.lidar_cloud = pointcloud2_2_npxyz(data)

    def camera_points2_callback(self, data):

        self.camera_cloud = pointcloud2_2_npxyz(data)

    def odom_callback(self, data):
        # print(data)
        position = data.pose.pose.position
        self.position = np.array([
            position.x,
            position.y,
            position.z
        ])

        orientation = data.pose.pose.orientation
        orientation = np.array([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        self.R = Rotation.from_quat(orientation)
        # try:
        #     self.R = Rotation.from_quat(orientation)
        # except:
        #     self.R = Rotation.from_quat((1, 0, 0, 0))
        #     # self.R = Rotation.from_euler(0, 0, 0)
        self.orientation = euler_from_quaternion(orientation)
        # print(data.pose)

    def detect_crest0(self):
        if self.camera_cloud is None: return
        if self.position is None: return
        if self.R is None: return

        points = self.camera_cloud.squeeze()
        pose = self.position
        R = self.R

        cpoints = R.apply(points) - pose
        height_threshold = 0.5
        th = 1
        # x_nnan, y_nnan = np.nonzero(~np.isnan(cloud[:, :, 0]))
        # points = cloud[x_nnan, y_nnan, :]
        angles = np.arctan2(cpoints[:, 1], cpoints[:, 0])
        ang_mask = np.logical_and(angles < np.pi / 6, angles > -np.pi/6)
        fpoints = cpoints[ang_mask, :]
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
        # mask = np.logical_or(diffx > th, diffz > height_threshold)
        mask = diffz > height_threshold
        # mask = np.abs(sorted_z) > 1
        # mask = np.logical_or(diffz > height_threshold,  np.abs(sorted_z[:-1]) > 1)
        idxs = np.nonzero(mask)[0]
        if len(idxs) > 0:
            d = sorted_x[idxs[0]]
        else:
            d = sorted_x[-1]

        print(d)
        self.d = d
        # return d

    def detect_crest1(self):
        if self.camera_cloud is None: return
        if self.position is None: return
        if self.R is None: return

        z_th = 1
        xdiff_th = 1
        max_dist = 20

        points = self.camera_cloud.squeeze()
        points = points[points[:, 0] < max_dist, :]
        # plt.hist(points[:, 1])
        # plt.show()
        pose = self.position
        R = self.R
        # print(pose)
        # print(self.orientation)
        cpoints = R.apply(points) - pose
        # y valuesx are all negative
        
        # plt.hist(cpoints[:, 1])
        # plt.show()

        # x_nnan, y_nnan = np.nonzero(~np.isnan(cloud[:, :, 0]))
        # points = cloud[x_nnan, y_nnan, :]
        angles = np.arctan2(cpoints[:, 1], cpoints[:, 0])
        # plt.hist(angles)
        # plt.show()
        # plt.hist(angles*180/np.pi, bins=15)
        # plt.show()
        ang_mask = np.logical_and(angles < np.pi / 6, angles > -np.pi/6)
        fpoints = cpoints[ang_mask, :]
        angles = angles[ang_mask]
        # print(fpoints[:, -1])
        # height_mask = np.logical_and(fpoints[:, -1] < height_threshold, fpoints[:, -1] > -height_threshold)
        # fpoints = fpoints[height_mask, :]

        args = np.argsort(fpoints[:, 0])
        sorted_x = fpoints[args, 0]
        sorted_z = fpoints[args, -1]
        sorted_points = fpoints[args, :]
        
        # angles = np.arctan2(fpoints[:, 1], fpoints[:, 0])

        step = 0.02
        samples = np.arange( - np.pi / 6 + 0.1, np.pi / 6 - 0.1, step)
        last_points = []
        xdiff = sorted_points[1:, 0] - sorted_points[:-1, 0]
        buff = np.zeros(shape=(len(sorted_points,)))
        buff[1:] = xdiff
        xdiff = buff
        for s in samples:
            inf = s
            sup = s + step
            mask1 = np.logical_and(
                angles > inf, 
                angles < sup
                )
            # print(np.sum(mask1))
            mask2 = np.logical_and(
                np.abs(sorted_points[:, -1]) < z_th,
                xdiff < xdiff_th
                )
            mask = np.logical_and(mask1, mask2)

            idxs = np.nonzero(mask)[0]
            # print(idxs)
            if len(idxs) > 0:
                last = sorted_points[idxs[-1], :]
                last_points.append(last)
            else:
                last_points.append(sorted_points[-1])
            # else:
            #     # last = -1
            #     try:
            #         idxs = np.nonzero(mask1)[0]
            #         last = sorted_points[idxs[-1]]
            #         last_points.append(last)
        
            #     except IndexError:
            #         pass
                    
        self.crest_cloud = last_points
            # last = sorted_points[idxs[-1], :]
            
        dists = [np.sqrt(np.sum(p ** 2)) for p in last_points]

        # d = min(dists)
        hist, bin_edges = np.histogram(dists, bins=5)
        # print(bin_edges)
        idx = np.argmax(hist)
        d = (bin_edges[idx] + bin_edges[idx + 1]) / 2




        # pca = PCA(n_components=1)
        # scaler = StandardScaler()
        # trans = pca.fit_transform(scaler.fit_transform(sorted_points))
        # diff = trans[1:] - trans[:-1]
        # # mask = diff > np.median(trans)
        # mask = np.abs(diff) > 1
        # idxs = np.nonzero(mask)[0]
        # if len(idxs) > 0:
        #     p =  trans[idxs[0]]
        # else:
        #     p = trans[-1]
        # p = pca.inverse_transform(p)
        # p = scaler.inverse_transform(p)
        # d = np.sqrt(np.sum((p - pose) ** 2))

        print(d)
        self.d = d

        # plt.hist(np.abs(diff), bins=50)
        # plt.scatter(np.arange(len(diff)), np.abs(diff))
        # plt.show()


    def detect_crest2(self):

        z_th = 1
        xdiff_th = 2
        max_dist = 30

        points = self.camera_cloud.squeeze()
        points = points[points[:, 0] < max_dist, :]
        pose = self.position
        R = self.R
        cpoints = R.apply(points) - pose

        cpoints = cpoints[cpoints[:, 0] < max_dist]

        angles = np.arctan2(cpoints[:, 1], cpoints[:, 0])
        ang_mask = np.logical_and(angles < np.pi / 6, angles > -np.pi/6)
        fpoints = cpoints[ang_mask, :]
        angles = angles[ang_mask]

        
        step = 3.0/180.0 * np.pi
        dstep = 0.5
        ang_samples = np.arange( - np.pi / 6.0 + step, np.pi / 6.0 - step, step)
        last_points = []
        
        for ang in ang_samples:
            inf = ang # inferior limit
            sup = ang + step # supperior limit
            # filter only angles in the range
            mask1 = np.logical_and(
                angles > inf, 
                angles < sup
                )

            if np.sum(mask1) == 0:
                continue

            rpoints = fpoints[mask1]
            dists = np.sqrt(np.sum(rpoints ** 2, axis=-1))
            # print(np.isnan(dists).sum())
            bins = np.arange(dists.min(), dists.max(), dstep)

            new_points = []
            for b in bins[:-1]:
                mask = np.logical_and(dists >= b, dists < b + step)
                if np.sum(mask) == 0:
                    continue
                p = rpoints[mask].mean(axis=0)
                new_points.append(p)
            
            if len(new_points) == 0:
                continue

            
            rpoints = np.array(new_points)
            # print(rpoints.shape)
            dists = np.sqrt(np.sum(rpoints ** 2, axis=-1))
            args = np.argsort(dists)
            sorted_points = rpoints[args]


            xdiff = sorted_points[1:, 0] - sorted_points[:-1, 0]

            maskx = xdiff > xdiff_th

            # maskz = np.abs(sorted_points[:, -1]) > z_th
            
            idxs = np.nonzero(maskx)[0]

            zdiff = sorted_points[1:, -1] - sorted_points[:-1, -1]
            slope = np.arctan2(zdiff, xdiff)
            masks = np.abs(slope) > 30.0/180.0 * np.pi

            if np.sum(maskx) > 0:
                i = idxs[0]
                if np.sum(masks[:i]) > 0:
                    i = np.nonzero(masks)[0][0]
                
                p = sorted_points[i]
                 
                # if np.sum(maskz[:i]) > 0:
                    
                #     zdiff = sorted_points[1:, -1] - sorted_points[:-1, -1]
                #     slope = np.arctan2(zdiff, xdiff)
                #     masks = np.abs(slope) > 10.0/180.0 * np.pi
                    
                #     if np.sum(masks) == 0:
                #         p = sorted_points[i]
                #     else:
                #         i = np.nonzero(maskx)[0][0]
                #         p = sorted_points[i]
                
                # else:
                #     p = sorted_points[i]
            
            else:
                p = sorted_points[-1]
            
            last_points.append(p)
                    
        self.crest_cloud = last_points
            # last = sorted_points[idxs[-1], :]
            
        dists = [np.sqrt(np.sum(p ** 2)) for p in last_points]

        # d = min(dists)
        # hist, bin_edges = np.histogram(dists, bins=5)
        # print(bin_edges)
        # idx = np.argmax(hist)
        # d = (bin_edges[idx] + bin_edges[idx + 1]) / 2

        d = np.min(dists)
        print(d)
        self.d = d


    def detect_crest3(self):

        z_th = 1
        xdiff_th = 2
        max_dist = 30

        points = self.camera_cloud.squeeze()
        points = points[points[:, 0] < max_dist, :]
        pose = self.position
        R = self.R
        cpoints = R.apply(points) - pose

        cpoints = cpoints[cpoints[:, 0] < max_dist]

        angles = np.arctan2(cpoints[:, 1], cpoints[:, 0])
        ang_mask = np.logical_and(angles < np.pi / 6, angles > -np.pi/6)
        fpoints = cpoints[ang_mask, :]
        angles = angles[ang_mask]

        max_angle_dev = 30
        inter_max_dev = 0.80
        max_dist = 30
        n_jobs = 8
        n_planes = 3

        cloud = fpoints

        remov_points = cloud[(cloud[:, 0] > max_dist), :]
        cloud = cloud[cloud[:, 0] < max_dist, :]
        X = cloud[:, 0:2]
        Y = cloud[:, -1]

        Xn = X
        Yn = Y
        planes = []
        lr = linear_model.LinearRegression(n_jobs=n_jobs)
        ransac = linear_model.RANSACRegressor(
            lr, residual_threshold=0.1 )
        for _ in range(n_planes):
            if Xn.shape[0] < 5:
                break
            ransac.fit(Xn, Yn)
            mask = ransac.inlier_mask_
            points = np.hstack((Xn[mask, :], Yn[mask].reshape(-1, 1)))
            coefs = (-ransac.estimator_.coef_[0], -ransac.estimator_.coef_[1], 1)
            planes.append(
                Plane(points,  coefs, -ransac.estimator_.intercept_)
            )
            Xn = Xn[~ mask, :]
            Yn = Yn[~ mask]


        # print(planes)

        ds = []
        candidates = []
        excluded = []
        for plane in planes:
            is_valid_ang = plane.angled > (90 - max_angle_dev) and (plane.angled < (90 + max_angle_dev)) 
            if (np.abs(plane.inter) < inter_max_dev) and is_valid_ang:
                candidates.append(plane)
            else:
                excluded.append(plane)

        # print(len(candidates))
        if len(candidates) > 1:
            ds = [np.sqrt(np.min(np.sum(p.points**2, axis=-1))) for p in candidates]
            arg = ds.index(min(ds))
            ground_plane = candidates[arg]
            del candidates[arg]
            excluded.extend(candidates)
        elif len(candidates) == 1:
            ground_plane = candidates[0]
                
        obst_points = np.vstack([p.points for p in excluded] + [remov_points])
        
        dists = np.sqrt(np.sum(ground_plane.points ** 2, axis=-1))
        d = np.max(dists)
        print(d)
        self.crest_cloud = ground_plane.points
        self.d = d


    def publish(self):

        if self.camera_cloud is None: return
        if self.position is None: return
        if self.R is None: return

        if self.alg_num == 0:
            self.detect_crest0()
        elif self.alg_num == 1:
            self.detect_crest1()
        elif self.alg_num == 2:
            self.detect_crest2()
        elif self.alg_num == 3:
            self.detect_crest3()
        
        if self.crest is not None:
            self.pub_crest.publish(self.crest)
        if self.crest_cloud is not None:
            header = Header()
            header.frame_id = 'base_link'
            header.seq = self.seq
            self.seq += 1
            header.stamp = rospy.Time.now()
            crest_cloud_msg = pc2.create_cloud_xyz32(header, self.crest_cloud)
            self.pub_crest_cloud.publish(crest_cloud_msg)
        

        # print(self.crest_cloud)


        dmsg = Float32()
        dmsg.data = self.d
        self.pub_dist.publish(dmsg)

    def run(self):

        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            rate.sleep()
            self.publish()

def main():
    argv = rospy.myargv(argv=sys.argv)
    
    if len(argv) == 2:
        alg_num = int(argv[1])
    elif len(argv) == 1:
        alg_num = 0
    else:
        raise ValueError('This node expects only one argument.')
    
    rospy.init_node('distance_camera', anonymous=True)

    # distcamera = DistanceCamera(camera_cloud_topic='/bumblebee2/point_cloud_transformed')
    distcamera = CameraDistOdom(
        edge_topic='/rtabmap/edge_distance',
        crest_topic='/rtabmap/crest',
        crest_cloud_topic='/rtabmap/crest_cloud',
        camera_cloud_topic='/rtabmap/cloud_ground',
        alg_num=alg_num)
    distcamera.run()
    

if __name__ == '__main__':
    main()