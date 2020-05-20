#!/usr/bin/env python
import _fix
import sys
import rospy
from gazebo_msgs.srv import DeleteModel
import random
import sys
import math

from safer_stack.utils import create_modelstate_message
from std_srvs.srv import Empty


def delete(model_name):
    rospy.wait_for_service('/gazebo/delete_model')

    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        # msg = DeleteModel()
        # msg.model_name
        resp = delete_model(model_name)
        print(resp)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) != 2:
        raise ValueError('You should pass one argument.')
   
    model_name = argv[1]

    delete(model_name)