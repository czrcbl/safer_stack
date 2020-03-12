#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np

from utils import create_message

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
    distances = []
    for item in data.detections:
        # print(item.pose.pose)
        position = item.pose.pose.pose.position
        x, y, z = position.x, position.y, position.z
        d = np.sqrt(x**2 + y**2 + z**2)
        print(d)
        distances.append(d)
        if d < 3:
            twist = create_message([0, 0, 0])
            pub.publish(twist)

def main():
    global pub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('StopHusky', anonymous=True)
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
