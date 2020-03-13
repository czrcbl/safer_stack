#!/usr/bin/python
import rospy 
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from utils import create_message


def callback(data):
    # global twist
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
        d = 1000
    print(d)
    if d < 3:
        twist = create_message([0, 0, 0])
    else:
        twist = create_message([1, 0, 0])
    pub.publish(twist)


def main():

    global pub

    rospy.init_node('ControlHusky', anonymous=True)
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback)
    rate = rospy.Rate(10)
    twist = create_message([1, 0, 0])
    pub.publish(twist)
    while not rospy.is_shutdown():
        # rospy.loginfo('Message sent!')
        rate.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass