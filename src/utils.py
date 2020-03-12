from geometry_msgs.msg import Twist, Vector3 

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