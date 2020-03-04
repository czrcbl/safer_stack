#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy

def callback(data):
    data = ros_numpy.numpify(data)

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.astype(int)[0])
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listen_lidar', anonymous=True)

    rospy.Subscriber("/velodyne_points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()