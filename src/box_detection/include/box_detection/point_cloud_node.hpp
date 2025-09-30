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
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bounding_boxes_sub_;
  
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr box_pointclouds_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plane_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr box_faces_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr box_poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_coordinate_frames_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr box_pose_image_pub_;

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
  
  // Bounding boxes data
  std::string current_bounding_boxes_;
  bool has_bounding_boxes_;
  
  // Camera intrinsics
  double fx_, fy_, cx_, cy_;
  std_msgs::msg::Header current_header_;

  // Parameters
  std::string target_frame_;
  std::string camera_frame_;
  int noise_filter_neighbors_;
  double noise_filter_std_dev_;
  
  // RANSAC parameters
  double ransac_distance_threshold_;
  int ransac_max_iterations_;
  double plane_area_threshold_;
  
  // Known box dimensions (add your actual box dimensions here)
  struct KnownBoxDimensions {
    double dim_a;  // Shortest dimension
    double dim_b;  // Medium dimension  
    double dim_c;  // Longest dimension
    std::string box_type;
  };
  
  std::vector<KnownBoxDimensions> known_boxes_;
  
  struct DetectedFace {
    std::string face_type;     // "(a,b)", "(a,c)", or "(b,c)"
    double length1, length2;   // Visible dimensions
    double perpendicular_dim;  // Hidden dimension
    double confidence;         // Match confidence
    std::string normal_direction; // "TOP", "FRONT", etc.
  };

  // Callback functions
  void colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  void boundingBoxesCallback(const std_msgs::msg::String::ConstSharedPtr& msg);
  
  // Main processing function
  void generate3DPointCloud();
  
  // Box-specific point cloud extraction
  void extractBoxPointClouds();
  
  // Noise removal function
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyRANSACNoiseRemoval(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);
    
  // RANSAC plane segmentation function
  void segmentPlanesInBoxes(
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& box_clouds,
    const std::vector<cv::Rect>& bounding_boxes);
    
  // Enhanced face detection with known dimensions
  DetectedFace determineFaceType(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& box_cloud,
    const std::string& normal_direction);
    
  // Estimate box dimensions from point cloud
  KnownBoxDimensions estimateBoxDimensions(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& box_cloud);
  
  // Match detected dimensions to known boxes
  std::pair<KnownBoxDimensions, double> matchToKnownBox(const KnownBoxDimensions& detected_dims);
  
  // Initialize known box dimensions
  void initializeKnownBoxes();
    
  // 6D pose estimation function
  void estimate6DPoseAndVisualize(
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& box_clouds,
    const std::vector<cv::Rect>& bounding_boxes);
    
  // Draw coordinate systems on image
  void drawCoordinateSystemsOnImage(
    const std::vector<Eigen::Vector3f>& centroids,
    const std::vector<Eigen::Matrix3f>& rotations);
    
  // Enhanced 6D pose estimation helper functions
  Eigen::Matrix3f estimateOrientationWithFaceConstraints(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
    const DetectedFace& detected_face);
    
  Eigen::Matrix3f estimateOrientationWithPCA(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud);
    
  Eigen::Vector3f measureBoxDimensions(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
    const Eigen::Matrix3f& rotation_matrix);
    
  Eigen::Matrix3f alignAxesWithDimensions(
    const Eigen::Matrix3f& pca_rotation,
    const Eigen::Vector3f& measured_dims,
    const Eigen::Vector3f& expected_dims);
    
  Eigen::Vector3f refineCentroidWithGeometry(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
    const Eigen::Vector3f& initial_centroid,
    const Eigen::Matrix3f& rotation_matrix,
    const Eigen::Vector3f& expected_dimensions);
    
  // Temporary function to save image with coordinates - COMMENTED OUT
  /*
  void saveImageWithCoordinates(
    const std::vector<Eigen::Vector3f>& centroids,
    const std::vector<Eigen::Matrix3f>& rotations,
    const std::string& filename_prefix = "pose_detection");
    
  // Temporary function: Save RGB image with coordinate annotations
  void saveAnnotatedImage(
    const std::vector<Eigen::Vector3f>& centroids,
    const std::vector<Eigen::Matrix3f>& rotations,
    const std::string& filename_prefix = "annotated_image");
  */
};

} // namespace box_detection

#endif // BOX_DETECTION_POINT_CLOUD_NODE_HPP_
