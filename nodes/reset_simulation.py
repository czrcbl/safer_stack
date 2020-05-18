#!/usr/bin/env python
import sys
import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import random
import sys
import math

from std_srvs.srv import Empty


def create_message(coords, yaw):

    pose = Pose()
    p = Point()
    p.x = coords[0]
    p.y = coords[1]
    p.z = coords[2]
    pose.position = p
    
    qua = quaternion_from_euler(0, 0, yaw)
    q = Quaternion()
    q.x = qua[0]
    q.y = qua[1]
    q.z = qua[2]
    q.w = qua[3]
    pose.orientation = q
    
    twist = Twist()
    twist.linear = Vector3(0, 0, 0)
    twist.angular = Vector3(0, 0, 0)

    ms = ModelState()

    ms.model_name = '/'
    ms.pose = pose
    ms.twist = twist
    ms.reference_frame = 'sand_mine'

    return ms

def change_pose(coords, yaw):
    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        msg = create_message(coords, yaw)
        resp = set_model_state(msg)
        print(resp)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def reset_map():

    rospy.wait_for_service('rtabmap/reset')
    reset = rospy.ServiceProxy('rtabmap/reset', Empty)
    resp = reset()
    print(resp)

if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) > 1:
        r = True
    else:
        r = False
    am = math.pi / 180
    default_yaw = 90 # 90 radians
    default_coords = (14.501486, -42.408237, 17.819097)
    if not r:
        change_pose(default_coords, default_yaw)
    else:
        coords = (random.uniform(2.0, 16.0), default_coords[1], default_coords[2])
        yaw = random.uniform(default_yaw - am * 25, default_yaw + am * 25)
        change_pose(coords, yaw)

    reset_map()