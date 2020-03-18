#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from utils import create_message


class HuskyControl:

    def __init__(self):
        self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/edge_distance', Float32, self.callback)
        self.twist = None
    
    def callback(self, data):
        d = data.data
        th = 4
        vel = max([min([d - th, 1]), 0.2])
        if np.isnan(d):
            twist = create_message([1, 0, 0])
        elif d < 4:
            twist = create_message([0, 0, 0])
        else:
            twist = create_message([vel, 0, 0])

        self.twist = twist

    def update(self):
        if self.twist is not None:
            self.pub.publish(self.twist)

def main():

    
    rospy.init_node('ControlHusky', anonymous=True)
    
    huskyctl = HuskyControl()
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