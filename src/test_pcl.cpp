// #include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// void callback(const PointCloud::ConstPtr& msg)
// {
//   printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
// //   BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//     // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "test_pcl");
//   ros::NodeHandle nh;
//   ros::Subscriber sub = nh.subscribe<PointCloud>("/bumblebee2/points2", 1, callback);
//   ros::spin();
// }

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Algorithms
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// C++ includes
#include <iostream>

ros::Publisher pub;


// sensor_msgs::PointCloud2 remove_outliers(const sensor_msgs::PointCloud2ConstPtr& input) 
// {
//   pcl::PCLPointCloud2<pcl::PointXYZ>* cloud = new pcl::PCLPointCloud2;
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2<pcl::PointXYZ> cloud_filtered;

//   pcl_conversions::toPCL(*input, *cloud);

//   // Create the filtering object
//   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//   sor.setInputCloud (cloud);
//   sor.setMeanK (50);
//   sor.setStddevMulThresh (1.0);
//   sor.filter (cloud_filtered);

//   sensor_msgs::PointCloud2 output;
//   pcl_conversions::fromPCL(cloud_filtered, output);

//   return output;

// }

sensor_msgs::PointCloud2 voxel(const sensor_msgs::PointCloud2ConstPtr& input) 
{
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  pcl_conversions::toPCL(*input, *cloud);
  
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  return output;
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  // output = *input;
  output = voxel(input);
  // output = remove_outliers(input);

  std::cout << "Output width: " << output.width << "\n" << "Output heigth: " << output.height << "\n";
  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_test");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/bumblebee2/points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/test_pcl/output", 1);

  // Spin
  ros::spin ();
}