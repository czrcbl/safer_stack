#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// C++ includes
#include <iostream>

typedef pcl::PointXYZRGB PointT;

ros::Publisher pub;

sensor_msgs::PointCloud2 down_sample(const sensor_msgs::PointCloud2ConstPtr& input) 
{
    pcl::PointCloud<PointT> cloud;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);

    int scale = 4;

    pcl::PointCloud<PointT> down_sampled_cloud; 
    down_sampled_cloud.width = cloud.width / scale;
    down_sampled_cloud.height = cloud.height / scale;
    std::cout << cloud.width << "\n";
    for( int ii = 0; ii < cloud.height; ii+=scale){
        for( int jj = 0; jj < cloud.width; jj+=scale ){
          // std::cout << ii << "\n";
          // std::cout << jj << "\n\n";
          down_sampled_cloud.push_back(cloud.at(jj, ii));
    }
}
    pcl::PCLPointCloud2 buffer;
    sensor_msgs::PointCloud2 output;
    pcl::toPCLPointCloud2(down_sampled_cloud, buffer);
    pcl_conversions::fromPCL(buffer, output);

    return output;
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  // output = *input;
  
  output = down_sample(input);
  // output = remove_statistical_outliers(output)
  // output = test(input);
  // output = remove_outliers(input);

  // std::cout << "Output width: " << output.width << "\n" << "Output heigth: " << output.height << "\n";
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
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/bumblebee2/points2_downsampled", 1);

  // Spin
  ros::spin ();
}