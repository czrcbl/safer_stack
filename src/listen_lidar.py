#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import cv2
import matplotlib.pyplot as plt
import numpy as np


def callback(data):
    pass
    # data = ros_numpy.numpify(data)

    # xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data) # Convert pointclud message to unstructured x,y,z coordinates
    # coords_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", coords_array[0, :])
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", coords_array.shape)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listen_lidar', anonymous=True)

    rospy.Subscriber("/velodyne_points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # cv2.imshow('Depth Image')
    
    rospy.spin()

if __name__ == '__main__':
    listener()