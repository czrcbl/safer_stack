#!/usr/bin/env python
import __fix
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 

from lib.utils import create_message

def talker():

    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('Move', anonymous=True)
    rate = rospy.Rate(10)
    
    twist = create_message([1,0,0])
    
    
    while not rospy.is_shutdown():
        # rospy.loginfo('Message sent!')
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass