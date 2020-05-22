#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from safer_stack.utils import create_twist_message
import sys


class HuskyControl:

    def __init__(self, edge_distance_topic='/edge_distance'):
        self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(edge_distance_topic, Float32, self.callback)
        self.twist = None
    
    def callback(self, data):
        d = data.data
        th = rospy.get_param('stop_distance', 7)
        vel = max([min([d - th, 1]), 0.2])
        if np.isnan(d):
            twist = create_twist_message([1, 0, 0])
        elif d < th:
            twist = create_twist_message([0, 0, 0])
        else:
            twist = create_twist_message([vel, 0, 0])

        self.twist = twist

    def update(self):
        if self.twist is not None:
            self.pub.publish(self.twist)

def main():
    argv = rospy.myargv(argv=sys.argv)
    dev = argv[1]

    rospy.init_node('ControlHusky', anonymous=True)
    
    huskyctl = HuskyControl(edge_distance_topic='/{}/edge_distance'.format(dev))
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # rospy.loginfo('Message sent!')
        huskyctl.update()
        rate.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass