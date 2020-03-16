#!/usr/bin/python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import ros_numpy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
bridge = CvBridge()


def callback(data_img, data_cloud):
    
    h = data_cloud.height
    w = data_cloud.width
    
    coords = np.zeros(shape=(h, w, 3))
    for n, point in enumerate(pc2.read_points(data_cloud, skip_nans=False)):
        # print(point[0]) 
        j = n % w
        i = n // w
        coords[i, j, :] = point[:3]

    # print(np.sum(np.isnan(coords)))
    coords[np.isnan(coords)] = 1000
    depth = np.sqrt((coords ** 2).sum(axis=-1))
    # print(depth.min(), depth.mean(), depth.max())
    # kernel = np.ones((5,5),np.float32)/25
    # depth = cv2.filter2D(depth,-1,kernel)
    th = 10
    max_dist = 20
    diff_arr = np.abs(depth[1:,:] - depth[:-1,:])
    buff = np.zeros(shape=depth.shape)
    buff[1:, :] = diff_arr
    # mask_arr = np.logical_and(buff > th, depth < max_dist)
    mask_arr = buff > th
    mask_img = mask_arr.astype(np.float32) 
    # print(mask_arr.sum())
    
    ## Process Image
    img = bridge.imgmsg_to_cv2(data_img, desired_encoding="passthrough")
    edges = cv2.Canny(img,50,200)
    
    # interzone = np.logical_and(edges, mask_img).astype('float')
    interzone = edges
    # rows, cols = np.nonzero(interzone)
    # height = int(np.median(rows))
    
    distances = []
    # for i in range(depth.shape[1]):
    #     distances.append(depth[height, i])
    for d in depth[interzone.astype(bool)]:
        distances.append(d)

    d = min(distances)
    pub_dist.publish(d)
    # print('closest point', min(distances))
    
    cv2.imshow('Mask', interzone)
    cv2.waitKey(3)

    
def listener():

    global pub_dist
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('DistanceCloud', anonymous=True)

    # rospy.Subscriber("/bumblebee2/points2", PointCloud2, callback)
    pub_dist = rospy.Publisher('/edge_distance', Float32, queue_size=10)
    image_sub = message_filters.Subscriber('/bumblebee2/left/image_rect', Image)
    cloud_sub = message_filters.Subscriber('/bumblebee2/points2', PointCloud2)

    ts = message_filters.TimeSynchronizer([image_sub, cloud_sub], 10)
    ts.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    # cv2.imshow('Depth Image')
    
    rospy.spin()

if __name__ == '__main__':
    listener()