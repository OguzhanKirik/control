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
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
// TF disabled - moved to dedicated point_cloud_node
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
  //rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr boundingBox_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr boundingBox_coordinates_pub_;

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


  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pointcloud_pub_;

  // Callback functions
  void colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  //void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg); // restored clustering version

  void detect2DBoxes(); 

  // TEMPORARY: Save 2D box detection image to local computer - COMMENTED OUT
  // void saveDetectionImage(const cv::Mat& annotated_image, const std::string& filename_prefix = "2d_detection"); 

};

} // namespace box_detection

#endif // BOX_DETECTION_BOX_DETECTOR_HPP_
