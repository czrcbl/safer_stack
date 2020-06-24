#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header
from safer_stack.utils import pointcloud2_2_npxyz
from sklearn.cluster import KMeans

seq = 0
def callback(data):
    global seq
    max_objects = 5
    cloud = pointcloud2_2_npxyz(data)
    cloud = cloud.squeeze()
    kms = []
    inertias = []
    for n_obj in range(2, max_objects + 1):
        km = KMeans(n_clusters=n_obj)
        km.fit(cloud)
        inertia = km.inertia_
        kms.append(km)
        inertias.append(inertia)
        
    idx = inertias.index(min(inertias))
    km = kms[idx]
    clusters = km.labels_
    print(clusters)
    out_clouds = []
    print(cloud.shape)
    for i in range(idx + 2):
        out_clouds.append(cloud[clusters == i, :])
    
    print('Number of obstacles:', idx + 2)
    # print(out_clouds)
    header = Header()
    header.frame_id = 'base_link'
    header.seq = seq
    seq += 1
    header.stamp = rospy.Time.now()
    crest_cloud_msg = pc2.create_cloud_xyz32(header, out_clouds[0])
    pub_obst.publish(crest_cloud_msg) 
    

if __name__ == '__main__':
    
    rospy.init_node('obstacle_segmenter', anonymous=True)
    rospy.Subscriber('/rtabmap/cloud_obstacles', PointCloud2, callback)
    pub_obst = rospy.Publisher('/obstacles', PointCloud2, queue_size=10)
    
    rospy.spin()