#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_py as tf2
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudTransformer:

    def __init__(self, input_topic, output_topic, target_frame):
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.target_frame = target_frame
        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber(input_topic, PointCloud2, self.callback, queue_size=2)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

    def callback(self, msg):
        lookup_time = msg.header.stamp
        target_frame = self.target_frame
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

        self.pub.publish(cloud_out)


# class TransformPointCloud:
#     def __init__(self, only_lidar=False, only_camera=False):
#         self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
#         self.tl = tf2_ros.TransformListener(self.tf_buffer)
        
        
#         if not only_lidar:
#             self.pub_camera = rospy.Publisher("output_camera", PointCloud2, queue_size=2)
#             self.sub_camera = rospy.Subscriber("input_camera", PointCloud2,
#                                         self.camera_callback, queue_size=2)

#         if not only_camera:
#             self.pub_lidar = rospy.Publisher("output_lidar", PointCloud2, queue_size=2)
#             self.sub_lidar = rospy.Subscriber("input_lidar", PointCloud2,
#                                         self.lidar_callback, queue_size=2)

#     def convert_point_cloud(self, msg):
#         lookup_time = msg.header.stamp
#         target_frame = 'base_link'
#         source_frame = msg.header.frame_id
#         try:
#             trans = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time,
#                                                     rospy.Duration(1))
#         except tf2.LookupException as ex:
#             rospy.logwarn(str(lookup_time.to_sec()))
#             rospy.logwarn(ex)
#             return
#         except tf2.ExtrapolationException as ex:
#             rospy.logwarn(str(lookup_time.to_sec()))
#             rospy.logwarn(ex)
#             return
#         cloud_out = do_transform_cloud(msg, trans)

#         return cloud_out
    
#     def camera_callback(self, msg):
#         cloud_out = self.convert_point_cloud(msg)
#         self.pub_camera.publish(cloud_out)
    
#     def lidar_callback(self, msg):
#         cloud_out = self.convert_point_cloud(msg)
#         self.pub_lidar.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node('transform_point_cloud', anonymous=True)
    only_lidar = rospy.get_param('only_lidar', False)
    only_camera = rospy.get_param('only_lidar', False)
    target_frame = rospy.get_param('only_lidar', 'base_link')

    if not only_camera:
        trans_lidar = PointCloudTransformer(
            input_topic='input_lidar', 
            output_topic='output_lidar',
            target_frame=target_frame)
    if not only_lidar:
        trans_camera = PointCloudTransformer(
            input_topic='input_camera', 
            output_topic='output_camera',
            target_frame=target_frame)

    # transform_point_cloud = TransformPointCloud(only_lidar=only_lidar, only_camera=only_camera)
    rospy.spin()