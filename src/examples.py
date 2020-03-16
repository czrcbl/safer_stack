# cv2 bridge
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
np_image = np.asarray(cv_image)

# ros_numpy and cv2
coords_array = ros_numpy.point_cloud2.pointcloud2_to_array(cloud).astype(float)
cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
imgray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(cv_image,50,500)
coords = coords_array[edges]
dists = np.sum(coords**2, axis=1)
rospy.init_node('listen_camera', anonymous=True)
cloud_sub = message_filters.Subscriber('/bumblebee2/points2', PointCloud2)
image_sub = message_filters.Subscriber('/bumblebee2/left/image_raw', Image)
ts = message_filters.TimeSynchronizer([cloud_sub, image_sub], 10)
ts.registerCallback(callback)
rospy.spin()

# LIDAR Seg
xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data) # Convert pointclud message to unstructured x,y,z coordinates


    data = ros_numpy.numpify(data)

    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data) # Convert pointclud message to unstructured x,y,z coordinates
    print(xyz_array.shape)