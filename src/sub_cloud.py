#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import sensor_msgs.point_cloud2 as pc2
import numpy as np
from utils import pointcloud2_2_npxyz

import roslib.message
from sensor_msgs.msg import PointCloud2, PointField


class CloudSub:

    def __init__(self, input_topic, output_topic, window=(8, 8)):
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.window = window
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2,
                                        self.cloud_callback, queue_size=2)

    def cloud_callback(self, msg):

        cloud = pointcloud2_2_npxyz(msg)
        
        height, width = cloud.shape[:-1]
        wh, ww = self.window[0], self.window[1]

        out = np.zeros(shape=(height//wh, width/ww, 3))
        for i in range(0, height, wh):
            for j in range(0, width, ww):
                x_nnan, y_nnan = np.nonzero(~np.isnan(cloud[i:i + wh, j:j + ww, 0]))
                if len(x_nnan) == 0:
                    p = (np.nan, np.nan, np.nan)
                else:
                    points = cloud[x_nnan, y_nnan, :]
                    p = np.mean(points, axis=0)
                # print(points)
                out[i//wh, j//ww, :] = p
        print(out)
        header = Header()
        header.stamp = rospy.Time.now()
        cloud_msg = create_cloud_xyz32(header, out)
        print(cloud_msg)

    def run(self):
        rospy.spin()

    
    
if __name__ == '__main__':

    rospy.init_node('sub_cloud', anonymous=True)
    cs = CloudSub(input_topic='/bumblebee2/points2', output_topic='/bumblebee2/points2_sub')
    cs.run()