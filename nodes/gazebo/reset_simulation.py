#!/usr/bin/env python
import _fix
import sys
import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import random
import sys
import math

from safer_stack.utils import create_modelstate_message
from std_srvs.srv import Empty


def change_pose(coords, yaw):
    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        msg = create_modelstate_message(coords, yaw)
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