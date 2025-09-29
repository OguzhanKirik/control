#include "box_detection/point_cloud_node.hpp"
#include <Eigen/Dense>
#include <pcl/filters/statistical_outlier_removal.h>

namespace box_detection {

PointCloudNode::PointCloudNode() : Node("point_cloud_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting Point Cloud Node");

  has_color_image_ = false;
  has_depth_image_ = false;
  has_camera_info_ = false;
  first_image_logged_ = false;

  // Parameters
  this->declare_parameter("target_frame", std::string("root_link"));
  this->declare_parameter("camera_frame", std::string("camera_color_optical_frame"));
  this->declare_parameter("noise_filter_neighbors", 50);
  this->declare_parameter("noise_filter_std_dev", 1.0);
  
  target_frame_ = this->get_parameter("target_frame").as_string();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
  noise_filter_neighbors_ = this->get_parameter("noise_filter_neighbors").as_int();
  noise_filter_std_dev_ = this->get_parameter("noise_filter_std_dev").as_double();

  // Camera info subscription
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", 10,
    std::bind(&PointCloudNode::cameraInfoCallback, this, std::placeholders::_1));

  // Publisher for 3D point cloud
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_node/pointcloud", 10);
  pointcloud_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_node/pointcloudfiltered", 10);
  
  // Initialize TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize image transport after a short delay to ensure node is fully constructed
  auto timer = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { initializeImageTransport(); });
}

PointCloudNode::~PointCloudNode() {}

void PointCloudNode::initializeImageTransport()
{
  try {
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    color_image_sub_ = it_->subscribe("/camera/color/image_raw", 1, 
                                     std::bind(&PointCloudNode::colorImageCallback, this, std::placeholders::_1));
    depth_image_sub_ = it_->subscribe("/camera/aligned_depth_to_color/image_raw", 1, 
                                     std::bind(&PointCloudNode::depthImageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Image transport initialized");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to init image transport: %s", e.what());
  }
}

void PointCloudNode::colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    current_color_image_ = cv_ptr->image.clone();
    current_header_ = msg->header;
    has_color_image_ = true;
    
    if (!first_image_logged_) {
      RCLCPP_INFO(this->get_logger(), "First color image received: %dx%d", 
                  current_color_image_.cols, current_color_image_.rows);
      first_image_logged_ = true;
    }
    
    // Generate point cloud when both images and camera info are available
    if (has_depth_image_ && has_camera_info_) {
      generate3DPointCloud();
    }

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge color error: %s", e.what());
  }
}

void PointCloudNode::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    current_depth_image_ = cv_ptr->image.clone();
    has_depth_image_ = true;
    
    // Generate point cloud when both images and camera info are available
    if (has_color_image_ && has_camera_info_) {
      generate3DPointCloud();
    }

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge depth error: %s", e.what());
  }
}

void PointCloudNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
  fx_ = msg->k[0]; 
  fy_ = msg->k[4]; 
  cx_ = msg->k[2]; 
  cy_ = msg->k[5];
  has_camera_info_ = true;
  
  static bool logged = false;
  if (!logged) {
    RCLCPP_INFO(this->get_logger(), "Camera info received - fx=%.1f fy=%.1f cx=%.1f cy=%.1f", 
                fx_, fy_, cx_, cy_);
    logged = true;
  }
}

void PointCloudNode::generate3DPointCloud()
{
    // Check if we have all required data
    if (!has_color_image_ || !has_depth_image_ || !has_camera_info_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Missing data - color:%d depth:%d camera_info:%d", 
                            (int)has_color_image_, (int)has_depth_image_, (int)has_camera_info_);
        return;
    }
    
    if (current_color_image_.empty() || current_depth_image_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Empty images - color:%s depth:%s", 
                            current_color_image_.empty() ? "empty" : "ok",
                            current_depth_image_.empty() ? "empty" : "ok");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Starting point cloud generation - Image size: %dx%d", 
                current_depth_image_.cols, current_depth_image_.rows);

    try {
        // Check TF transform from target frame to camera frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            // Use TimePointZero for rosbag replay to get latest available transform
            transform_stamped = tf_buffer_->lookupTransform(
                target_frame_, camera_frame_, 
                tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            return;
        }

        // Create PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        cv::Mat color_image = current_color_image_;
        cv::Mat depth_image = current_depth_image_;
        
        // Set cloud properties
        cloud->width = depth_image.cols;
        cloud->height = depth_image.rows;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);
        
        // Generate point cloud
        int valid_points = 0;
        int total_points = 0;
        uint16_t min_depth = 65535, max_depth = 0;
        
        for (int v = 0; v < depth_image.rows; ++v) {
            for (int u = 0; u < depth_image.cols; ++u) {
                int index = v * depth_image.cols + u;
                pcl::PointXYZRGB& point = cloud->points[index];
                total_points++;
                
                // Get depth value (assuming depth is in millimeters)
                uint16_t depth_mm = depth_image.at<uint16_t>(v, u);
                
                // Track depth statistics
                if (depth_mm > 0) {
                    if (depth_mm < min_depth) min_depth = depth_mm;
                    if (depth_mm > max_depth) max_depth = depth_mm;
                }
                
                if (depth_mm == 0 || depth_mm > 10000) { // Filter out invalid/too far points
                    // Invalid depth
                    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
                    point.r = point.g = point.b = 0;
                } else {
                    // Convert depth to meters
                    float depth_m = static_cast<float>(depth_mm) / 1000.0f;
                    
                    // Project to 3D using camera intrinsics (camera coordinate system)
                    point.x = (u - cx_) * depth_m / fx_;
                    point.y = (v - cy_) * depth_m / fy_;
                    point.z = depth_m;
                    
                    // Get color information
                    if (u < color_image.cols && v < color_image.rows) {
                        cv::Vec3b color_pixel = color_image.at<cv::Vec3b>(v, u);
                        point.b = color_pixel[0];  // OpenCV uses BGR
                        point.g = color_pixel[1];
                        point.r = color_pixel[2];
                    } else {
                        point.r = point.g = point.b = 128;  // Gray for missing color
                    }
                    valid_points++;
                }
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Point cloud stats - Valid: %d/%d, Depth range: %d-%d mm", 
                    valid_points, total_points, min_depth, max_depth);
        
        // Transform point cloud to target frame
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // Create transformation matrix from TF
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        
        // Translation
        transform(0,3) = transform_stamped.transform.translation.x;
        transform(1,3) = transform_stamped.transform.translation.y;
        transform(2,3) = transform_stamped.transform.translation.z;
        
        // Rotation (quaternion to rotation matrix)
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
            
        tf2::Matrix3x3 rot_matrix(q);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                transform(i,j) = rot_matrix[i][j];
            }
        }
        
        // Apply transformation
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
        
        // Apply noise removal filters
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = applyStatisticalNoiseRemoval(transformed_cloud);
        
        // Convert to ROS message and publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*transformed_cloud, output_msg);
        output_msg.header = current_header_;
        output_msg.header.frame_id = target_frame_;
    
  
        sensor_msgs::msg::PointCloud2 output_filtered_msg;
        pcl::toROSMsg(*filtered_cloud, output_filtered_msg);
        output_filtered_msg.header = current_header_;
        output_filtered_msg.header.frame_id = target_frame_;
  

        pointcloud_pub_->publish(output_msg);
        pointcloud_filtered_pub_->publish(output_filtered_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Published 3D point cloud: %d valid points -> %zu filtered points (%.1f%% kept), frame: %s", 
                            valid_points, filtered_cloud->points.size(), 
                            (filtered_cloud->points.size() * 100.0) / valid_points, target_frame_.c_str());
                            
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in generate3DPointCloud: %s", e.what());
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudNode::applyStatisticalNoiseRemoval(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
    // Create output cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    try {
        // Check if input cloud is valid
        if (!input_cloud || input_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Input cloud is empty, skipping noise removal");
            return input_cloud;
        }
        
        // Statistical Outlier Removal
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(input_cloud);
        sor.setMeanK(noise_filter_neighbors_);
        sor.setStddevMulThresh(noise_filter_std_dev_);
        sor.filter(*filtered_cloud);
        
        // Log filtering results
        size_t original_size = input_cloud->points.size();
        size_t filtered_size = filtered_cloud->points.size();
        double removal_percentage = ((double)(original_size - filtered_size) / original_size) * 100.0;
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Statistical noise removal: %zu -> %zu points (%.1f%% removed)", 
                    original_size, filtered_size, removal_percentage);
        
        return filtered_cloud;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in statistical noise removal: %s", e.what());
        return input_cloud;  // Return original cloud if filtering fails
    }
}

} // namespace box_detection
