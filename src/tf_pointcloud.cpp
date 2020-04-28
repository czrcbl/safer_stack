#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <iostream>

class TransformPointCloud
{
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string target_frame_;
  double timeout_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  boost::recursive_mutex dr_mutex_;

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {

    // ROS_INFO_STREAM(msg->header.frame_id << " " << target_frame_);
    ROS_DEBUG_STREAM("input:");
    geometry_msgs::TransformStamped transform;
    try
    {
      // (target_frame, pc frame) preserves the world coordinates of the point cloud but shifts
      // the parent to target_frame_
      // (pc_frame, target_frame) shifts the point cloud to be relative to the target_frame
      // by the same amount it used to be relative to msg->header.frame_id,
      // but the frame will still be msg->header.frame_id when done.
      const std::string target_frame = "base_link";
    //   nh_.param("target_frame", target_frame, "base_link");
      const std::string source_frame = msg->header.frame_id;
    //   std::cout << source_frame << "\n";
      transform = tf_buffer_.lookupTransform(
          target_frame,
          source_frame,
          msg->header.stamp,
          ros::Duration(10));
      sensor_msgs::PointCloud2 cloud_out;
      tf2::doTransform(*msg, cloud_out, transform);
      ROS_DEBUG_STREAM("output:");
      // TODO(lwalter) should the scaling be done on the output?
      pub_.publish(cloud_out);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }

public:
  TransformPointCloud() :
    // nh_("~"),
    tf_listener_(tf_buffer_)
  {
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 3);
    sub_ = nh_.subscribe("input", 1, &TransformPointCloud::pointCloudCallback, this);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_point_cloud");
  TransformPointCloud transform_point_cloud;
  ros::spin();
}