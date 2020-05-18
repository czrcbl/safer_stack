#!/usr/bin/env python
import roslib; roslib.load_manifest('laser_assembler')
import rospy; from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud2

rospy.init_node("assembler", anonymous=True)
# rospy.wait_for_service("hdl_floor_assembler")
rospy.wait_for_service('laser_assembler/AssembleScans2')
pub = rospy.Publisher('output', PointCloud2, queue_size=1)

while True:
    print ('while')
    try:
        assemble_scans = rospy.ServiceProxy('hdl_floor_assembler', AssembleScans2)
        resp = assemble_scans(rospy.get_rostime() - rospy.Time(10, 0), rospy.get_rostime())
        print "Got cloud with %u points" % len(resp.cloud.points)
        pub.publish(resp)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rospy.sleep(0.1)