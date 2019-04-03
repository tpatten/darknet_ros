#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// PCL
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
// OpenCV
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>

const std::string kCloudFrame = "camera_depth_optical_frame";
const std::string kRGBFrame = "camera_rgb_optical_frame";

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_publisher");
  ros::NodeHandle n("~");

  ROS_INFO("[cloud_publisher] Started");

  // Get the file
  std::string input_file;
  if (!n.getParam("filename", input_file)) {
    ROS_WARN("[cloud_publisher] Point cloud file not specified");
    ros::shutdown();
    return EXIT_FAILURE;
  }
  // Get the topic name to publish the points to
  std::string cloud_topic;
  if (!n.getParam("cloud_topic", cloud_topic)) {
    ROS_WARN("[cloud_publisher] Cloud topic name not specified");
    ros::shutdown();
    return EXIT_FAILURE;
  }
  // Get the topic name to publish the rgb image to
  std::string rgb_topic;
  if (!n.getParam("rgb_topic", rgb_topic)) {
    ROS_WARN("[cloud_publisher] RGB topic name not specified");
    ros::shutdown();
    return EXIT_FAILURE;
  }

  // Read the point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>());
  if (pcl::io::loadPCDFile(input_file, *cloud) != 0) {
    ROS_ERROR("[cloud_publisher] Error loading file '%s'", input_file.c_str());
    ros::shutdown();
    return EXIT_FAILURE;
  }
  ROS_INFO("[cloud_publisher] Point cloud from file '%s' has %lu points", input_file.c_str(), cloud->size() );
  if (cloud->empty()) {
    ROS_ERROR("[cloud_publisher] Point cloud is empty");
    ros::shutdown();
    return EXIT_FAILURE;
  }

  // Create the cloud and rgb image messages
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = kCloudFrame;
  cloud_msg.height = cloud->height;
  cloud_msg.width = cloud->width;
  cloud_msg.is_dense = cloud->isOrganized();
  sensor_msgs::Image rgb_msg;
  pcl::toROSMsg(cloud_msg, rgb_msg);

  // Create the publishers
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1);
  ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>(rgb_topic, 1);

  // Continually publish the point cloud and rgb image
  ros::Rate rate(5);
  ROS_INFO("[cloud_publisher] Publishing...");
  ros::Time time_stamp = ros::Time::now();
  while (ros::ok()) {
    time_stamp = ros::Time::now();
    cloud_msg.header.stamp = time_stamp;
    rgb_msg.header.stamp = time_stamp;
    cloud_pub.publish(cloud_msg);
    rgb_pub.publish(rgb_msg);
    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();
  return EXIT_SUCCESS;
}