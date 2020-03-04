#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 

def talker():

    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('Move', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        linear = Vector3()
        linear.x = 1
        linear.y = 0
        linear.z = 0
        angular = Vector3()
        angular.x = 0
        angular.y = 0
        angular.z = 0

        twist = Twist()
        twist.linear = linear
        twist.angular = angular

        pub.publish(twist)
        # rospy.loginfo('Message sent!')
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass