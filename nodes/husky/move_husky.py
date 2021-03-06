#!/usr/bin/env python
import _fix
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 

from safer_stack.utils import create_twist_message

def talker():

    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('Move', anonymous=True)
    rate = rospy.Rate(10)
    
    twist = create_twist_message([1,0,0])
    
    
    while not rospy.is_shutdown():
        # rospy.loginfo('Message sent!')
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass