#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from utils import create_message


def callback(data):
    global pub_dist
    distances = []
    for item in data.detections:
        # print(item.pose.pose)
        position = item.pose.pose.pose.position
        x, y, z = position.x, position.y, position.z
        d = np.sqrt(x**2 + y**2 + z**2)
        distances.append(d)
    
    if len(distances) > 0:
        d = min(distances)
    else:
        d = np.float('nan')

    pub_dist.publish(d)


def main():

    global pub_dist
    rospy.init_node('DistanceTag', anonymous=True)
    pub_dist = rospy.Publisher('/edge_distance', Float32, queue_size=10)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass