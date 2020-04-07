#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Algorithms
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>

// C++ includes
#include <iostream>

ros::Publisher pub;

// sensor_msgs::PointCloud2 remove_statistical_outliers(const sensor_msgs::PointCloud2ConstPtr& input) 
// {
//   pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2 cloud_filtered;

//   pcl_conversions::toPCL(*input, *cloud);

//   // Create the filtering object
//   pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
//   sor.setInputCloud(cloudPtr);
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


// sensor_msgs::PointCloud2 ground_plane(const sensor_msgs::PointCloud2ConstPtr& input) 
// {
//   pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2 cloud_filtered;

//   pcl_conversions::toPCL(*input, *cloud);

//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//   // Create the segmentation object
//   pcl::SACSegmentation<pcl::PCLPointCloud2> seg;
//   // Optional
//   seg.setOptimizeCoefficients (true);
//   // Mandatory
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setDistanceThreshold (0.01);

//   seg.setInputCloud (cloudPtr);
//   seg.segment (*inliers, *coefficients);

//   sensor_msgs::PointCloud2 output;
//   pcl_conversions::fromPCL(cloud_filtered, output);

//   return output;
// }



// sensor_msgs::PointCloud2 process_cloud(const sensor_msgs::PointCloud2ConstPtr& input)
// {
//   pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2 cloud_filtered;

//   pcl_conversions::toPCL(*input, *cloud);

//   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//   sor.setInputCloud (cloudPtr);
//   sor.setLeafSize (0.1, 0.1, 0.1);
//   sor.filter (cloud_filtered);

//   pcl::PCLPointCloud2ConstPtr filt_cloudPtr(new pcl::PCLPointCloud2);

//   pcl::fromPCLPointCloud2(cloud_filtered, *filt_cloudPtr);

//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//   // Create the segmentation object
//   pcl::SACSegmentation<pcl::PCLPointCloud2> seg;
//   // Optional
//   seg.setOptimizeCoefficients (true);
//   // Mandatory
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setDistanceThreshold (0.01);

//   seg.setInputCloud (filt_cloudPtr);
//   seg.segment (*inliers, *coefficients);


//   sensor_msgs::PointCloud2 output;
//   pcl_conversions::fromPCL(cloud_filtered, output);

//   return output;
// }

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  // output = *input;
  
  output = voxel(input);
  // output = remove_statistical_outliers(output)
  // output = test(input);
  // output = remove_outliers(input);

  std::cout << "Output width: " << output.width << "\n" << "Output heigth: " << output.height << "\n";
  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_test", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/bumblebee2/points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/bumblebee2/points2_voxel", 1);

  // Spin
  ros::spin ();
}