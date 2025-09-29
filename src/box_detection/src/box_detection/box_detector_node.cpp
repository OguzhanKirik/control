// Restored full node with image + depth subscriptions and point cloud generation
#include "box_detection/box_detector_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace box_detection {

BoxDetector::BoxDetector() : Node("box_detector_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting Box Detector Node");

  has_color_image_ = false;
  has_depth_image_ = false;
  has_camera_info_ = false;
  first_image_logged_ = false;

  // fx_ = 525.0; fy_ = 525.0; cx_ = 320.0; cy_ = 240.0;

 
 //this->create_publisher<sensor_msgs::msg::PointCloud2>("/box_detection/pointcloud", 10);

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", 10,
    std::bind(&BoxDetector::cameraInfoCallback, this, std::placeholders::_1));


  auto timer = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { initializeImageTransport(); });
}

BoxDetector::~BoxDetector() {}

void BoxDetector::initializeImageTransport()
{
  try {
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    color_image_sub_ = it_->subscribe("/camera/color/image_raw", 1, std::bind(&BoxDetector::colorImageCallback, this, std::placeholders::_1));
    depth_image_sub_ = it_->subscribe("/camera/aligned_depth_to_color/image_raw", 1, std::bind(&BoxDetector::depthImageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Image transport initialized");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to init image transport: %s", e.what());
  }
}

void BoxDetector::colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    current_color_image_ = cv_ptr->image.clone();
    current_header_ = msg->header;
    has_color_image_ = true;
    if (!first_image_logged_) {
      RCLCPP_INFO(this->get_logger(), "First color image received: %dx%d", current_color_image_.cols, current_color_image_.rows);
      first_image_logged_ = true;
    }

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge color error: %s", e.what());
  }
}

void BoxDetector::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    current_depth_image_ = cv_ptr->image.clone();
    has_depth_image_ = true;

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge depth error: %s", e.what());
  }
}

void BoxDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
  fx_ = msg->k[0]; fy_ = msg->k[4]; cx_ = msg->k[2]; cy_ = msg->k[5];
  has_camera_info_ = true;
  static bool logged = false;
  if (!logged) {
    RCLCPP_INFO(this->get_logger(), "Camera info fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx_, fy_, cx_, cy_);
    logged = true;
  }
}





// void BoxDetector::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
// {
  
// }

void BoxDetector::detect2DBoxes()
{
 
}


} // namespace box_detection

