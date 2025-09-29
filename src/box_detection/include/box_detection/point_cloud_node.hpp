#ifndef BOX_DETECTION_POINT_CLOUD_NODE_HPP_
#define BOX_DETECTION_POINT_CLOUD_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

// TF for transformation
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

namespace box_detection
{

class PointCloudNode : public rclcpp::Node
{
public:
  PointCloudNode();
  ~PointCloudNode();
  void initializeImageTransport();

private:
  // Subscribers
  image_transport::Subscriber color_image_sub_;
  image_transport::Subscriber depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_filtered_pub_;

  // TF components
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Image transport
  std::shared_ptr<image_transport::ImageTransport> it_;

  // Current images and data
  cv::Mat current_color_image_;
  cv::Mat current_depth_image_;
  bool has_color_image_;
  bool has_depth_image_;
  bool has_camera_info_;
  bool first_image_logged_;
  
  // Camera intrinsics
  double fx_, fy_, cx_, cy_;
  std_msgs::msg::Header current_header_;

  // Parameters
  std::string target_frame_;
  std::string camera_frame_;
  int noise_filter_neighbors_;
  double noise_filter_std_dev_;

  // Callback functions
  void colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  
  // Main processing function
  void generate3DPointCloud();
  
  // Noise removal function
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyStatisticalNoiseRemoval(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);
};

} // namespace box_detection

#endif // BOX_DETECTION_POINT_CLOUD_NODE_HPP_
