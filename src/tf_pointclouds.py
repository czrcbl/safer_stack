#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_py as tf2

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class TransformPointCloud:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)
        self.pub_camera = rospy.Publisher("/bumblebee2/point_cloud_transformed", PointCloud2, queue_size=2)
        self.pub_lidar = rospy.Publisher("/velodyne/point_cloud_transformed", PointCloud2, queue_size=2)
        self.sub_camera = rospy.Subscriber("/bumblebee2/points2", PointCloud2,
                                    self.camera_callback, queue_size=2)
        self.sub_lidar = rospy.Subscriber("/velodyne_points", PointCloud2,
                                    self.lidar_callback, queue_size=2)

    def convert_point_cloud(self, msg):
        lookup_time = msg.header.stamp
        target_frame = msg.header.frame_id 
        source_frame = msg.header.frame_id
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time,
                                                    rospy.Duration(1))
        except tf2.LookupException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        cloud_out = do_transform_cloud(msg, trans)

        return cloud_out
    
    def camera_callback(self, msg):
        cloud_out = self.convert_point_cloud(msg)
        self.pub_camera.publish(cloud_out)
    
    def lidar_callback(self, msg):
        cloud_out = self.convert_point_cloud(msg)
        self.pub_lidar.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node('transform_point_cloud')
    transform_point_cloud = TransformPointCloud()
    rospy.spin()