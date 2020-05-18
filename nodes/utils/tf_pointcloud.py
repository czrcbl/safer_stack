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
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1.0))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

    def callback(self, msg):
        lookup_time = msg.header.stamp
        target_frame = self.target_frame
        source_frame = msg.header.frame_id
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time,
                                                    rospy.Duration(0.0))
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

if __name__ == '__main__':
    rospy.init_node('transform_point_cloud', anonymous=True)
    target_frame = rospy.get_param('target_frame', 'base_link')
    trans = PointCloudTransformer(
            input_topic='input', 
            output_topic='output',
            target_frame=target_frame)
    rospy.spin()