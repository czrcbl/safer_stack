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

// Segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// C++ includes
#include <iostream>

ros::Publisher pub;


// sensor_msgs::PointCloud2 segment_obstacles(const sensor_msgs::PointCloud2ConstPtr& input) 
int segment_obstacles(const sensor_msgs::PointCloud2ConstPtr& input) 
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ> *cloud = new pcl::PointCloud<pcl::PointXYZ>;
    // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    // pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
   

    // pcl::fromPCLPointCloud2(*input, *cloud);
    pcl::fromROSMsg(*input, *cloud);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
    }

    return (0);
}






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

// sensor_msgs::PointCloud2 voxel(const sensor_msgs::PointCloud2ConstPtr& input) 
// {
//   pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2 cloud_filtered;

//   pcl_conversions::toPCL(*input, *cloud);

//   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//   sor.setInputCloud (cloudPtr);
//   sor.setLeafSize (0.1, 0.1, 0.1);
//   sor.filter (cloud_filtered);

//   sensor_msgs::PointCloud2 output;
//   pcl_conversions::fromPCL(cloud_filtered, output);

//   return output;
// }


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
    int out;
    out = segment_obstacles(input);
  // Do data processing here...
  // output = *input;
  
//   output = voxel(input);
  // output = remove_statistical_outliers(output)
  // output = test(input);
  // output = remove_outliers(input);

//   std::cout << "Output width: " << output.width << "\n" << "Output heigth: " << output.height << "\n";
  // Publish the data.
//   pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_test", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/rtabmap/cloud_obstacles", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/rtabmap/segmented_obstacles", 1);

  // Spin
  ros::spin ();
}