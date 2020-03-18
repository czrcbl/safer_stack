from geometry_msgs.msg import Twist, Vector3
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import os

project_path = os.path.abspath(os.path.realpath(os.path.dirname(os.path.dirname(__file__))))


def create_message(linear, angular=[0,0,0]):
    
    linearm = Vector3()
    linearm.x = linear[0]
    linearm.y = linear[1]
    linearm.z = linear[2]
    angularm = Vector3()
    angularm.x = angular[0]
    angularm.y = angular[1]
    angularm.z = angular[2]

    twist = Twist()
    twist.linear = linearm
    twist.angular = angularm

    return twist


def pointcloud2_2_npxyz(data):
    h = data.height
    w = data.width

    coords = np.zeros(shape=(h, w, 3))
    for n, point in enumerate(pc2.read_points(data, skip_nans=False)):
        # print(point[0]) 
        j = n % w
        i = n // w
        coords[i, j, :] = point[:3]
    
    return coords