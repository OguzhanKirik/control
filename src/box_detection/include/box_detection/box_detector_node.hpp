#ifndef BOX_DETECTION_BOX_DETECTOR_HPP_
#define BOX_DETECTION_BOX_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
// TF disabled
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>

namespace box_detection
{

class BoxDetector : public rclcpp::Node
{
public:
  BoxDetector();
  ~BoxDetector();
  void initializeImageTransport();

private:
  // Subscribers
  image_transport::Subscriber color_image_sub_;
  image_transport::Subscriber depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  // Publishers
  //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;

  // Image transport
  std::shared_ptr<image_transport::ImageTransport> it_;

  // Current images
  cv::Mat current_color_image_;
  cv::Mat current_depth_image_;
  bool has_color_image_;
  bool has_depth_image_;
  bool has_camera_info_;
  bool first_image_logged_;
  
  // Camera intrinsics
  double fx_, fy_, cx_, cy_;
  std_msgs::msg::Header current_header_;

  // TF related (disabled)
  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // std::string world_frame_ = "root_link";
  // std::string camera_optical_frame_ = "camera_color_optical_frame";
  // rclcpp::TimerBase::SharedPtr tf_timer_;
  // double floor_margin_m_ = 0.02;
  // bool floor_margin_param_declared_ = false;
  // geometry_msgs::msg::TransformStamped last_camera_tf_;
  // bool have_camera_tf_ = false;
  // void updateCameraPoseTF();

  int sync_tolerance_ms_ = 30;
  // Height filtering parameter
  double min_height_m_ = 1.8; // points below this Z will be removed

  // Clustering parameters
  double cluster_tolerance_m_ = 0.05; // Euclidean distance tolerance
  int cluster_min_size_ = 500;
  int cluster_max_size_ = 100000;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pointcloud_pub_;

  // Callback functions
  void colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  //void processImages();
  
  // sensor_msgs::msg::PointCloud2::SharedPtr createPointCloudFromDepth(
  //   const cv::Mat& depth_image, const std_msgs::msg::Header& header);
  // sensor_msgs::msg::PointCloud2::SharedPtr createPointCloudFromDepthColor(
  //   const cv::Mat& depth_image, const cv::Mat& color_image, const std_msgs::msg::Header& header);
};

} // namespace box_detection

#endif // BOX_DETECTION_BOX_DETECTOR_HPP_
