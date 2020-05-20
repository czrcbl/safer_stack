#!/usr/bin/env python
import _fix
import sys
import rospy
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from gazebo_msgs.srv import SetModelState
import random
import sys
import math

from safer_stack.utils import create_modelstate_message
from std_srvs.srv import Empty


def change_pose(coords, yaw, model_name):
    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        msg = create_modelstate_message(coords, yaw, model_name)
        resp = set_model_state(msg)
        print(resp)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) < 6:
        raise ValueError('Wrong number of arguments.')
    x = float(argv[1])
    y = float(argv[2])
    z = float(argv[3])
    w = float(argv[4]) # In degrees
    model_name = argv[5]
    am = math.pi / 180
    change_pose([x, y, z], w * am, model_name)