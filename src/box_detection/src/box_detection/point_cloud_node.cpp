#include "box_detection/point_cloud_node.hpp"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <sstream>

namespace box_detection {

PointCloudNode::PointCloudNode() : Node("point_cloud_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting Point Cloud Node");

  has_color_image_ = false;
  has_depth_image_ = false;
  has_camera_info_ = false;
  first_image_logged_ = false;
  has_bounding_boxes_ = false;

  // Parameters
  this->declare_parameter("target_frame", std::string("root_link"));
  this->declare_parameter("camera_frame", std::string("camera_color_optical_frame"));
  this->declare_parameter("noise_filter_neighbors", 50);
  this->declare_parameter("noise_filter_std_dev", 1.0);
  this->declare_parameter("ransac_distance_threshold", 0.01);
  this->declare_parameter("ransac_max_iterations", 1000);
  this->declare_parameter("plane_area_threshold", 0.001);
  
  target_frame_ = this->get_parameter("target_frame").as_string();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
  noise_filter_neighbors_ = this->get_parameter("noise_filter_neighbors").as_int();
  noise_filter_std_dev_ = this->get_parameter("noise_filter_std_dev").as_double();
  ransac_distance_threshold_ = this->get_parameter("ransac_distance_threshold").as_double();
  ransac_max_iterations_ = this->get_parameter("ransac_max_iterations").as_int();
  plane_area_threshold_ = this->get_parameter("plane_area_threshold").as_double();

  // Camera info subscription
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", 10,
    std::bind(&PointCloudNode::cameraInfoCallback, this, std::placeholders::_1));

  // Bounding boxes subscription
  bounding_boxes_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/box_detection/boxes_coordinates", 10,
    std::bind(&PointCloudNode::boundingBoxesCallback, this, std::placeholders::_1));

  // Publisher for 3D point cloud
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_node/pointcloud", 10);
  pointcloud_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_node/pointcloudfiltered", 10);
  box_pointclouds_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_node/box_pointclouds", 10);
  plane_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/point_cloud_node/plane_markers", 10);
  box_faces_pub_ = this->create_publisher<std_msgs::msg::String>("/point_cloud_node/box_faces", 10);
  box_poses_pub_ = this->create_publisher<std_msgs::msg::String>("/point_cloud_node/box_poses", 10);
  box_coordinate_frames_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/point_cloud_node/box_coordinate_frames", 10);
  box_pose_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/point_cloud_node/box_pose_image", 10);
  
  // Initialize TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize known box dimensions
  initializeKnownBoxes();

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

void PointCloudNode::boundingBoxesCallback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  current_bounding_boxes_ = msg->data;
  has_bounding_boxes_ = true;
  
  RCLCPP_DEBUG(this->get_logger(), "Received bounding boxes: %s", current_bounding_boxes_.c_str());
  
  // Extract box point clouds when we have all required data
  if (has_color_image_ && has_depth_image_ && has_camera_info_) {
    extractBoxPointClouds();
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

void PointCloudNode::extractBoxPointClouds()
{
    // Check if we have all required data
    if (!has_color_image_ || !has_depth_image_ || !has_camera_info_ || !has_bounding_boxes_) {
        RCLCPP_DEBUG(this->get_logger(), "Missing data for box point cloud extraction - color:%d depth:%d camera_info:%d bboxes:%d", 
                    (int)has_color_image_, (int)has_depth_image_, (int)has_camera_info_, (int)has_bounding_boxes_);
        return;
    }
    
    if (current_color_image_.empty() || current_depth_image_.empty() || current_bounding_boxes_.empty()) {
        return;
    }

    try {
        // Check TF transform from target frame to camera frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                target_frame_, camera_frame_, 
                tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                "Failed to get transform: %s", ex.what());
            return;
        }

        // Parse bounding boxes from string format "x,y,width,height;x2,y2,width2,height2;..."
        std::vector<cv::Rect> bounding_boxes;
        std::stringstream ss(current_bounding_boxes_);
        std::string box_str;
        
        while (std::getline(ss, box_str, ';')) {
            if (box_str.empty()) continue;
            
            std::stringstream box_ss(box_str);
            std::string coord;
            std::vector<int> coords;
            
            while (std::getline(box_ss, coord, ',')) {
                coords.push_back(std::stoi(coord));
            }
            
            if (coords.size() == 4) {
                cv::Rect bbox(coords[0], coords[1], coords[2], coords[3]);
                // Ensure bounding box is within image bounds
                bbox.x = std::max(0, bbox.x);
                bbox.y = std::max(0, bbox.y);
                bbox.width = std::min(bbox.width, current_depth_image_.cols - bbox.x);
                bbox.height = std::min(bbox.height, current_depth_image_.rows - bbox.y);
                
                if (bbox.width > 0 && bbox.height > 0) {
                    bounding_boxes.push_back(bbox);
                }
            }
        }
        
        if (bounding_boxes.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No valid bounding boxes found");
            return;
        }

        // Create combined point cloud for all detected boxes
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        combined_box_cloud->is_dense = false;
        
        // Store individual box clouds for plane segmentation
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> individual_box_clouds;
        
        cv::Mat color_image = current_color_image_;
        cv::Mat depth_image = current_depth_image_;
        
        // Define colors for different boxes
        std::vector<cv::Vec3b> box_colors = {
            cv::Vec3b(255, 0, 0),    // Red
            cv::Vec3b(0, 255, 0),    // Green  
            cv::Vec3b(0, 0, 255),    // Blue
            cv::Vec3b(255, 255, 0),  // Cyan
            cv::Vec3b(255, 0, 255),  // Magenta
            cv::Vec3b(0, 255, 255),  // Yellow
            cv::Vec3b(128, 0, 128),  // Purple
            cv::Vec3b(255, 165, 0)   // Orange
        };

        RCLCPP_INFO(this->get_logger(), "Processing %zu bounding boxes for 3D point extraction", bounding_boxes.size());

        for (size_t box_idx = 0; box_idx < bounding_boxes.size(); ++box_idx) {
            const cv::Rect& bbox = bounding_boxes[box_idx];
            cv::Vec3b box_color = box_colors[box_idx % box_colors.size()];
            
            // Create individual box cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr individual_box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            individual_box_cloud->is_dense = false;
            
            int valid_points_in_box = 0;
            
            // Extract 3D points within the bounding box region
            for (int v = bbox.y; v < bbox.y + bbox.height; ++v) {
                for (int u = bbox.x; u < bbox.x + bbox.width; ++u) {
                    // Get depth value (assuming depth is in millimeters)
                    uint16_t depth_mm = depth_image.at<uint16_t>(v, u);
                    
                    if (depth_mm == 0 || depth_mm > 10000) { // Filter out invalid/too far points
                        continue;
                    }
                    
                    // Convert depth to meters
                    float depth_m = static_cast<float>(depth_mm) / 1000.0f;
                    
                    // Project to 3D using camera intrinsics (camera coordinate system)
                    pcl::PointXYZRGB point;
                    point.x = (u - cx_) * depth_m / fx_;
                    point.y = (v - cy_) * depth_m / fy_;
                    point.z = depth_m;
                    
                    // Use box-specific color to distinguish different boxes
                    point.r = box_color[2];  // OpenCV uses BGR, PCL uses RGB
                    point.g = box_color[1];
                    point.b = box_color[0];
                    
                    combined_box_cloud->points.push_back(point);
                    individual_box_cloud->points.push_back(point);
                    valid_points_in_box++;
                }
            }
            
            // Set properties for individual box cloud
            if (!individual_box_cloud->points.empty()) {
                individual_box_cloud->width = individual_box_cloud->points.size();
                individual_box_cloud->height = 1;
                individual_box_clouds.push_back(individual_box_cloud);
            }
            
            RCLCPP_INFO(this->get_logger(), "Box %zu [%d,%d,%d,%d]: extracted %d 3D points", 
                       box_idx + 1, bbox.x, bbox.y, bbox.width, bbox.height, valid_points_in_box);
        }
        
        if (combined_box_cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No valid 3D points found in any bounding box");
            return;
        }

        // Update cloud properties
        combined_box_cloud->width = combined_box_cloud->points.size();
        combined_box_cloud->height = 1;

        // Transform point cloud to target frame
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
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
        
        // Apply transformation to individual box clouds and perform plane segmentation
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformed_individual_clouds;
        for (auto& box_cloud : individual_box_clouds) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_individual_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*box_cloud, *transformed_individual_cloud, transform);
            transformed_individual_clouds.push_back(transformed_individual_cloud);
        }
        
        // Perform plane segmentation on individual box clouds
        segmentPlanesInBoxes(transformed_individual_clouds, bounding_boxes);
        
        // Estimate 6D pose and visualize coordinate frames
        estimate6DPoseAndVisualize(transformed_individual_clouds, bounding_boxes);
        
        // Apply transformation to combined cloud
        pcl::transformPointCloud(*combined_box_cloud, *transformed_box_cloud, transform);
        
        // Apply noise removal filter
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_box_cloud = applyStatisticalNoiseRemoval(transformed_box_cloud);
        
        // Convert to ROS message and publish
        sensor_msgs::msg::PointCloud2 box_output_msg;
        pcl::toROSMsg(*filtered_box_cloud, box_output_msg);
        box_output_msg.header = current_header_;
        box_output_msg.header.frame_id = target_frame_;
        
        box_pointclouds_pub_->publish(box_output_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Published box point clouds: %zu total points from %zu boxes, frame: %s", 
                            filtered_box_cloud->points.size(), bounding_boxes.size(), target_frame_.c_str());
                            
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in extractBoxPointClouds: %s", e.what());
    }
}

void PointCloudNode::segmentPlanesInBoxes(
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& box_clouds,
    const std::vector<cv::Rect>& bounding_boxes)
{
    if (box_clouds.size() != bounding_boxes.size()) {
        RCLCPP_ERROR(this->get_logger(), "Mismatch between box clouds and bounding boxes count");
        return;
    }
    
    visualization_msgs::msg::MarkerArray marker_array;
    std::string face_detection_summary = "";
    
    for (size_t box_idx = 0; box_idx < box_clouds.size(); ++box_idx) {
        const auto& box_cloud = box_clouds[box_idx];
        
        if (box_cloud->points.empty()) {
            continue;
        }
        
        RCLCPP_INFO(this->get_logger(), "Analyzing box %zu with %zu points", box_idx + 1, box_cloud->points.size());
        
        try {
            // Apply noise removal first
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = applyStatisticalNoiseRemoval(box_cloud);
            
            if (filtered_cloud->points.size() < 50) {
                RCLCPP_WARN(this->get_logger(), "Box %zu: Too few points (%zu) for reliable plane detection", 
                           box_idx + 1, filtered_cloud->points.size());
                continue;
            }
            
            // RANSAC plane segmentation
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(ransac_max_iterations_);
            seg.setDistanceThreshold(ransac_distance_threshold_);
            
            seg.setInputCloud(filtered_cloud);
            seg.segment(*inliers, *coefficients);
            
            if (inliers->indices.empty()) {
                RCLCPP_WARN(this->get_logger(), "Box %zu: No plane found", box_idx + 1);
                continue;
            }
            
            // Extract the plane
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(filtered_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*plane_cloud);
            
            if (plane_cloud->points.empty()) {
                continue;
            }
            
            // Calculate plane area (rough estimation using bounding box of plane points)
            pcl::PointXYZRGB min_pt, max_pt;
            pcl::getMinMax3D(*plane_cloud, min_pt, max_pt);
            double plane_area = (max_pt.x - min_pt.x) * (max_pt.y - min_pt.y);
            
            if (plane_area < plane_area_threshold_) {
                RCLCPP_WARN(this->get_logger(), "Box %zu: Plane area too small (%.4f m²)", box_idx + 1, plane_area);
                continue;
            }
            
            // Get plane normal vector
            Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            normal.normalize();
            
            // Determine which face we're looking at based on normal vector
            std::string face_name;
            std::string face_description;
            
            // Compare normal with canonical directions (allowing some tolerance)
            double tolerance = 0.7; // cos(45°) ≈ 0.707
            
            if (std::abs(normal.z()) > tolerance) {
                if (normal.z() > 0) {
                    face_name = "TOP";
                    face_description = "Top face (normal pointing up)";
                } else {
                    face_name = "BOTTOM";
                    face_description = "Bottom face (normal pointing down)";
                }
            } else if (std::abs(normal.y()) > tolerance) {
                if (normal.y() > 0) {
                    face_name = "FRONT";
                    face_description = "Front face (normal pointing forward)";
                } else {
                    face_name = "BACK";
                    face_description = "Back face (normal pointing backward)";
                }
            } else if (std::abs(normal.x()) > tolerance) {
                if (normal.x() > 0) {
                    face_name = "RIGHT";
                    face_description = "Right face (normal pointing right)";
                } else {
                    face_name = "LEFT";
                    face_description = "Left face (normal pointing left)";
                }
            } else {
                face_name = "UNKNOWN";
                face_description = "Unknown orientation";
            }
            
            // Calculate plane centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*plane_cloud, centroid);
            
            // Enhanced face detection using known dimensions
            DetectedFace detected_face = determineFaceType(filtered_cloud, normal, face_name);
            
            RCLCPP_INFO(this->get_logger(), 
                       "Box %zu: %s - %.1f%% points in plane (area: %.4f m²)", 
                       box_idx + 1, face_description.c_str(),
                       (double)inliers->indices.size() / filtered_cloud->points.size() * 100.0,
                       plane_area);
                       
            RCLCPP_INFO(this->get_logger(),
                       "  🎯 Face type: %s (confidence: %.1f%%), visible dims: %.3fx%.3f m, hidden dim: %.3f m",
                       detected_face.face_type.c_str(), detected_face.confidence * 100.0,
                       detected_face.length1, detected_face.length2, detected_face.perpendicular_dim);
            
            // Add enhanced information to summary string
            if (!face_detection_summary.empty()) {
                face_detection_summary += ";";
            }
            face_detection_summary += "Box" + std::to_string(box_idx + 1) + ":" + face_name + 
                                    ":face_type:" + detected_face.face_type +
                                    ":confidence:" + std::to_string(detected_face.confidence) +
                                    ":visible_dims:" + std::to_string(detected_face.length1) + "x" + std::to_string(detected_face.length2) +
                                    ":hidden_dim:" + std::to_string(detected_face.perpendicular_dim) +
                                    ":area:" + std::to_string(plane_area) +
                                    ":normal:" + std::to_string(normal.x()) + "," + std::to_string(normal.y()) + "," + std::to_string(normal.z());
            
            // Create visualization marker for the plane with enhanced info
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = target_frame_;
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "box_faces";
            marker.id = box_idx;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2] + 0.05; // Slightly above the plane
            marker.pose.orientation.w = 1.0;
            
            marker.scale.z = 0.025; // Text size
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            
            marker.text = "Box" + std::to_string(box_idx + 1) + ": " + face_name;
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            
            marker_array.markers.push_back(marker);
            
            // Create arrow marker for normal vector
            visualization_msgs::msg::Marker arrow_marker;
            arrow_marker.header.frame_id = target_frame_;
            arrow_marker.header.stamp = this->get_clock()->now();
            arrow_marker.ns = "plane_normals";
            arrow_marker.id = box_idx;
            arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker.action = visualization_msgs::msg::Marker::ADD;
            
            arrow_marker.pose.position.x = centroid[0];
            arrow_marker.pose.position.y = centroid[1];
            arrow_marker.pose.position.z = centroid[2];
            
            // Convert normal vector to quaternion for arrow orientation
            Eigen::Vector3f z_axis(0, 0, 1);
            Eigen::Vector3f rotation_axis = z_axis.cross(normal);
            float rotation_angle = std::acos(z_axis.dot(normal));
            
            if (rotation_axis.norm() > 1e-6) {
                rotation_axis.normalize();
                tf2::Quaternion q;
                q.setRotation(tf2::Vector3(rotation_axis.x(), rotation_axis.y(), rotation_axis.z()), rotation_angle);
                arrow_marker.pose.orientation.x = q.x();
                arrow_marker.pose.orientation.y = q.y();
                arrow_marker.pose.orientation.z = q.z();
                arrow_marker.pose.orientation.w = q.w();
            } else {
                arrow_marker.pose.orientation.w = 1.0;
            }
            
            arrow_marker.scale.x = 0.1;  // Arrow length
            arrow_marker.scale.y = 0.01; // Arrow width
            arrow_marker.scale.z = 0.01; // Arrow height
            
            arrow_marker.color.a = 0.8;
            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 1.0;
            arrow_marker.color.b = 0.0;
            
            arrow_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            marker_array.markers.push_back(arrow_marker);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in plane segmentation for box %zu: %s", box_idx + 1, e.what());
        }
    }
    
    // Publish visualization markers
    if (!marker_array.markers.empty()) {
        plane_markers_pub_->publish(marker_array);
    }
    
    // Publish structured face detection data
    if (!face_detection_summary.empty()) {
        auto face_msg = std_msgs::msg::String();
        face_msg.data = face_detection_summary;
        box_faces_pub_->publish(face_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Published face detection summary: %s", face_detection_summary.c_str());
    }
}

void PointCloudNode::estimate6DPoseAndVisualize(
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& box_clouds,
    const std::vector<cv::Rect>& bounding_boxes)
{
    if (box_clouds.size() != bounding_boxes.size()) {
        RCLCPP_ERROR(this->get_logger(), "Mismatch between box clouds and bounding boxes count for pose estimation");
        return;
    }
    
    visualization_msgs::msg::MarkerArray coordinate_frames;
    std::string pose_data_summary = "";
    
    // Store pose data for image visualization
    std::vector<Eigen::Vector3f> box_centroids;
    std::vector<Eigen::Matrix3f> box_rotations;
    
    for (size_t box_idx = 0; box_idx < box_clouds.size(); ++box_idx) {
        const auto& box_cloud = box_clouds[box_idx];
        
        if (box_cloud->points.empty()) {
            continue;
        }
        
        try {
            // Apply noise removal first
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = applyStatisticalNoiseRemoval(box_cloud);
            
            if (filtered_cloud->points.size() < 30) {
                RCLCPP_WARN(this->get_logger(), "Box %zu: Too few points (%zu) for reliable pose estimation", 
                           box_idx + 1, filtered_cloud->points.size());
                continue;
            }
            
            // Calculate centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*filtered_cloud, centroid);
            
            // Calculate covariance matrix for PCA
            Eigen::Matrix3f covariance_matrix;
            pcl::computeCovarianceMatrixNormalized(*filtered_cloud, centroid, covariance_matrix);
            
            // Eigen decomposition to get principal axes
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
            Eigen::Matrix3f eigenVectors = eigen_solver.eigenvectors();
            Eigen::Vector3f eigenValues = eigen_solver.eigenvalues();
            
            // Sort eigenvectors by eigenvalues (largest to smallest)
            std::vector<std::pair<float, int>> eigen_pairs;
            for (int i = 0; i < 3; ++i) {
                eigen_pairs.push_back(std::make_pair(eigenValues(i), i));
            }
            std::sort(eigen_pairs.begin(), eigen_pairs.end(), std::greater<std::pair<float, int>>());
            
            // Reorder eigenvectors: largest eigenvalue -> longest axis (Y), smallest -> shortest axis (X)
            Eigen::Vector3f x_axis = eigenVectors.col(eigen_pairs[2].second); // Shortest axis (X)
            Eigen::Vector3f y_axis = eigenVectors.col(eigen_pairs[0].second); // Longest axis (Y) 
            Eigen::Vector3f z_axis = eigenVectors.col(eigen_pairs[1].second); // Medium axis (Z)
            
            // Ensure right-handed coordinate system
            if (x_axis.cross(y_axis).dot(z_axis) < 0) {
                z_axis = -z_axis;
            }
            
            // Calculate box dimensions by projecting points onto principal axes
            float min_x = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float min_y = std::numeric_limits<float>::max();
            float max_y = std::numeric_limits<float>::lowest();
            float min_z = std::numeric_limits<float>::max();
            float max_z = std::numeric_limits<float>::lowest();
            
            for (const auto& point : filtered_cloud->points) {
                Eigen::Vector3f pt(point.x - centroid[0], point.y - centroid[1], point.z - centroid[2]);
                float proj_x = pt.dot(x_axis);
                float proj_y = pt.dot(y_axis);
                float proj_z = pt.dot(z_axis);
                
                min_x = std::min(min_x, proj_x);
                max_x = std::max(max_x, proj_x);
                min_y = std::min(min_y, proj_y);
                max_y = std::max(max_y, proj_y);
                min_z = std::min(min_z, proj_z);
                max_z = std::max(max_z, proj_z);
            }
            
            float width = max_x - min_x;   // X dimension (shorter edge)
            float length = max_y - min_y;  // Y dimension (longer edge)
            float height = max_z - min_z;  // Z dimension
            
            // Create rotation matrix from axes
            Eigen::Matrix3f rotation_matrix;
            rotation_matrix.col(0) = x_axis;
            rotation_matrix.col(1) = y_axis;
            rotation_matrix.col(2) = z_axis;
            
            // Store pose data for image visualization
            box_centroids.push_back(Eigen::Vector3f(centroid[0], centroid[1], centroid[2]));
            box_rotations.push_back(rotation_matrix);
            
            // Convert to quaternion
            Eigen::Quaternionf quaternion(rotation_matrix);
            quaternion.normalize();
            
            // Log pose information
            RCLCPP_INFO(this->get_logger(), 
                       "Box %zu 6D Pose - Position: [%.3f, %.3f, %.3f], "
                       "Orientation: [%.3f, %.3f, %.3f, %.3f], "
                       "Dimensions: %.3fx%.3fx%.3f m (WxLxH)",
                       box_idx + 1,
                       centroid[0], centroid[1], centroid[2],
                       quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w(),
                       width, length, height);
            
            // Build pose data string
            if (!pose_data_summary.empty()) {
                pose_data_summary += ";";
            }
            pose_data_summary += "Box" + std::to_string(box_idx + 1) + 
                               ":pos:" + std::to_string(centroid[0]) + "," + std::to_string(centroid[1]) + "," + std::to_string(centroid[2]) +
                               ":quat:" + std::to_string(quaternion.x()) + "," + std::to_string(quaternion.y()) + "," + 
                               std::to_string(quaternion.z()) + "," + std::to_string(quaternion.w()) +
                               ":dim:" + std::to_string(width) + "," + std::to_string(length) + "," + std::to_string(height);
            
            // Create coordinate frame visualization
            // X-axis (red) - shorter edge
            visualization_msgs::msg::Marker x_marker;
            x_marker.header.frame_id = target_frame_;
            x_marker.header.stamp = this->get_clock()->now();
            x_marker.ns = "box_coordinate_frames";
            x_marker.id = box_idx * 3;
            x_marker.type = visualization_msgs::msg::Marker::ARROW;
            x_marker.action = visualization_msgs::msg::Marker::ADD;
            
            x_marker.pose.position.x = centroid[0];
            x_marker.pose.position.y = centroid[1];
            x_marker.pose.position.z = centroid[2];
            
            // Create quaternion for X-axis direction
            Eigen::Vector3f default_x(1, 0, 0);
            Eigen::Vector3f rotation_axis_x = default_x.cross(x_axis);
            float rotation_angle_x = std::acos(default_x.dot(x_axis));
            
            if (rotation_axis_x.norm() > 1e-6) {
                rotation_axis_x.normalize();
                tf2::Quaternion q_x;
                q_x.setRotation(tf2::Vector3(rotation_axis_x.x(), rotation_axis_x.y(), rotation_axis_x.z()), rotation_angle_x);
                x_marker.pose.orientation.x = q_x.x();
                x_marker.pose.orientation.y = q_x.y();
                x_marker.pose.orientation.z = q_x.z();
                x_marker.pose.orientation.w = q_x.w();
            } else {
                x_marker.pose.orientation.w = 1.0;
            }
            
            x_marker.scale.x = 0.05; // Arrow length
            x_marker.scale.y = 0.005; // Arrow width
            x_marker.scale.z = 0.005; // Arrow height
            x_marker.color.a = 1.0;
            x_marker.color.r = 1.0; // Red for X
            x_marker.color.g = 0.0;
            x_marker.color.b = 0.0;
            x_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            
            coordinate_frames.markers.push_back(x_marker);
            
            // Y-axis (green) - longer edge
            visualization_msgs::msg::Marker y_marker = x_marker;
            y_marker.id = box_idx * 3 + 1;
            
            Eigen::Vector3f default_y(0, 1, 0);
            Eigen::Vector3f rotation_axis_y = default_y.cross(y_axis);
            float rotation_angle_y = std::acos(default_y.dot(y_axis));
            
            if (rotation_axis_y.norm() > 1e-6) {
                rotation_axis_y.normalize();
                tf2::Quaternion q_y;
                q_y.setRotation(tf2::Vector3(rotation_axis_y.x(), rotation_axis_y.y(), rotation_axis_y.z()), rotation_angle_y);
                y_marker.pose.orientation.x = q_y.x();
                y_marker.pose.orientation.y = q_y.y();
                y_marker.pose.orientation.z = q_y.z();
                y_marker.pose.orientation.w = q_y.w();
            } else {
                y_marker.pose.orientation.w = 1.0;
            }
            
            y_marker.color.r = 0.0;
            y_marker.color.g = 1.0; // Green for Y
            y_marker.color.b = 0.0;
            
            coordinate_frames.markers.push_back(y_marker);
            
            // Z-axis (blue)
            visualization_msgs::msg::Marker z_marker = x_marker;
            z_marker.id = box_idx * 3 + 2;
            
            Eigen::Vector3f default_z(0, 0, 1);
            Eigen::Vector3f rotation_axis_z = default_z.cross(z_axis);
            float rotation_angle_z = std::acos(default_z.dot(z_axis));
            
            if (rotation_axis_z.norm() > 1e-6) {
                rotation_axis_z.normalize();
                tf2::Quaternion q_z;
                q_z.setRotation(tf2::Vector3(rotation_axis_z.x(), rotation_axis_z.y(), rotation_axis_z.z()), rotation_angle_z);
                z_marker.pose.orientation.x = q_z.x();
                z_marker.pose.orientation.y = q_z.y();
                z_marker.pose.orientation.z = q_z.z();
                z_marker.pose.orientation.w = q_z.w();
            } else {
                z_marker.pose.orientation.w = 1.0;
            }
            
            z_marker.color.r = 0.0;
            z_marker.color.g = 0.0;
            z_marker.color.b = 1.0; // Blue for Z
            
            coordinate_frames.markers.push_back(z_marker);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in 6D pose estimation for box %zu: %s", box_idx + 1, e.what());
        }
    }
    
    // Publish coordinate frame visualization
    if (!coordinate_frames.markers.empty()) {
        box_coordinate_frames_pub_->publish(coordinate_frames);
    }
    
    // Draw coordinate systems on camera image and publish
    if (!box_centroids.empty()) {
        drawCoordinateSystemsOnImage(box_centroids, box_rotations);
    }
    
    // Publish pose data
    if (!pose_data_summary.empty()) {
        auto pose_msg = std_msgs::msg::String();
        pose_msg.data = pose_data_summary;
        box_poses_pub_->publish(pose_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Published 6D pose data for %zu boxes", box_clouds.size());
    }
}

void PointCloudNode::drawCoordinateSystemsOnImage(
    const std::vector<Eigen::Vector3f>& centroids,
    const std::vector<Eigen::Matrix3f>& rotations)
{
    if (current_color_image_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No color image available for coordinate system visualization");
        return;
    }
    
    try {
        // Get current transform from camera to target frame (inverse of what we used before)
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                camera_frame_, target_frame_, 
                tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                "Failed to get transform for image projection: %s", ex.what());
            return;
        }
        
        // Create transformation matrix from target frame back to camera frame
        Eigen::Matrix4f camera_transform = Eigen::Matrix4f::Identity();
        
        // Translation
        camera_transform(0,3) = transform_stamped.transform.translation.x;
        camera_transform(1,3) = transform_stamped.transform.translation.y;
        camera_transform(2,3) = transform_stamped.transform.translation.z;
        
        // Rotation (quaternion to rotation matrix)
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
            
        tf2::Matrix3x3 rot_matrix(q);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                camera_transform(i,j) = rot_matrix[i][j];
            }
        }
        
        // Clone the original image for drawing
        cv::Mat pose_image = current_color_image_.clone();
        
        // Define axis colors (RGB format for OpenCV)
        cv::Scalar x_color(0, 0, 255);    // Red for X-axis
        cv::Scalar y_color(0, 255, 0);    // Green for Y-axis  
        cv::Scalar z_color(255, 0, 0);    // Blue for Z-axis
        
        float axis_length = 0.05f; // 5cm axis length
        int line_thickness = 3;
        
        for (size_t box_idx = 0; box_idx < centroids.size(); ++box_idx) {
            const Eigen::Vector3f& centroid = centroids[box_idx];
            const Eigen::Matrix3f& rotation = rotations[box_idx];
            
            // Transform centroid and axes from target frame to camera frame
            Eigen::Vector4f centroid_homo(centroid[0], centroid[1], centroid[2], 1.0f);
            Eigen::Vector4f centroid_camera = camera_transform * centroid_homo;
            
            // Calculate axis endpoints in target frame
            Eigen::Vector3f x_end = centroid + rotation.col(0) * axis_length;
            Eigen::Vector3f y_end = centroid + rotation.col(1) * axis_length;
            Eigen::Vector3f z_end = centroid + rotation.col(2) * axis_length;
            
            // Transform axis endpoints to camera frame
            Eigen::Vector4f x_end_homo(x_end[0], x_end[1], x_end[2], 1.0f);
            Eigen::Vector4f y_end_homo(y_end[0], y_end[1], y_end[2], 1.0f);
            Eigen::Vector4f z_end_homo(z_end[0], z_end[1], z_end[2], 1.0f);
            
            Eigen::Vector4f x_end_camera = camera_transform * x_end_homo;
            Eigen::Vector4f y_end_camera = camera_transform * y_end_homo;
            Eigen::Vector4f z_end_camera = camera_transform * z_end_homo;
            
            // Project 3D points to 2D image coordinates
            auto project3DTo2D = [this](const Eigen::Vector4f& point_3d) -> cv::Point {
                if (point_3d[2] <= 0) return cv::Point(-1, -1); // Behind camera
                
                float x_2d = (point_3d[0] * fx_) / point_3d[2] + cx_;
                float y_2d = (point_3d[1] * fy_) / point_3d[2] + cy_;
                
                return cv::Point(static_cast<int>(x_2d), static_cast<int>(y_2d));
            };
            
            cv::Point center_2d = project3DTo2D(centroid_camera);
            cv::Point x_end_2d = project3DTo2D(x_end_camera);
            cv::Point y_end_2d = project3DTo2D(y_end_camera);
            cv::Point z_end_2d = project3DTo2D(z_end_camera);
            
            // Check if points are within image bounds
            auto isPointValid = [&pose_image](const cv::Point& pt) -> bool {
                return pt.x >= 0 && pt.y >= 0 && pt.x < pose_image.cols && pt.y < pose_image.rows;
            };
            
            if (!isPointValid(center_2d)) {
                continue; // Skip if center is outside image
            }
            
            // Draw coordinate axes
            if (isPointValid(x_end_2d)) {
                cv::arrowedLine(pose_image, center_2d, x_end_2d, x_color, line_thickness, 8, 0, 0.3);
                cv::putText(pose_image, "X", x_end_2d + cv::Point(5, 5), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, x_color, 2);
            }
            
            if (isPointValid(y_end_2d)) {
                cv::arrowedLine(pose_image, center_2d, y_end_2d, y_color, line_thickness, 8, 0, 0.3);
                cv::putText(pose_image, "Y", y_end_2d + cv::Point(5, 5), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, y_color, 2);
            }
            
            if (isPointValid(z_end_2d)) {
                cv::arrowedLine(pose_image, center_2d, z_end_2d, z_color, line_thickness, 8, 0, 0.3);
                cv::putText(pose_image, "Z", z_end_2d + cv::Point(5, 5), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, z_color, 2);
            }
            
            // Draw box center point
            cv::circle(pose_image, center_2d, 5, cv::Scalar(255, 255, 255), -1);
            cv::circle(pose_image, center_2d, 6, cv::Scalar(0, 0, 0), 2);
            
            // Draw box label
            std::string box_label = "Box" + std::to_string(box_idx + 1);
            cv::Point label_pos = center_2d + cv::Point(10, -10);
            cv::putText(pose_image, box_label, label_pos, 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            cv::putText(pose_image, box_label, label_pos, 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 1);
        }
        
        // Add legend
        int legend_x = 10;
        int legend_y = 30;
        int legend_spacing = 25;
        
        cv::putText(pose_image, "Coordinate Systems:", cv::Point(legend_x, legend_y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        cv::putText(pose_image, "Coordinate Systems:", cv::Point(legend_x, legend_y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
        
        cv::arrowedLine(pose_image, cv::Point(legend_x, legend_y + legend_spacing), 
                       cv::Point(legend_x + 30, legend_y + legend_spacing), x_color, 2, 8, 0, 0.3);
        cv::putText(pose_image, "X (short edge)", cv::Point(legend_x + 35, legend_y + legend_spacing + 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, x_color, 2);
        
        cv::arrowedLine(pose_image, cv::Point(legend_x, legend_y + 2*legend_spacing), 
                       cv::Point(legend_x + 30, legend_y + 2*legend_spacing), y_color, 2, 8, 0, 0.3);
        cv::putText(pose_image, "Y (long edge)", cv::Point(legend_x + 35, legend_y + 2*legend_spacing + 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, y_color, 2);
        
        cv::arrowedLine(pose_image, cv::Point(legend_x, legend_y + 3*legend_spacing), 
                       cv::Point(legend_x + 30, legend_y + 3*legend_spacing), z_color, 2, 8, 0, 0.3);
        cv::putText(pose_image, "Z (height)", cv::Point(legend_x + 35, legend_y + 3*legend_spacing + 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, z_color, 2);
        
        // Convert to ROS message and publish
        auto img_msg = cv_bridge::CvImage(current_header_, sensor_msgs::image_encodings::BGR8, pose_image).toImageMsg();
        box_pose_image_pub_->publish(*img_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Published pose visualization image with %zu coordinate systems", centroids.size());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in drawCoordinateSystemsOnImage: %s", e.what());
    }
}

// Initialize known box dimensions - YOUR ACTUAL BOX DIMENSIONS
void PointCloudNode::initializeKnownBoxes() 
{
    known_boxes_.clear();
    
    // Actual box dimensions from your data (sorted as a <= b <= c)
    // Small box: [0.340, 0.250, 0.095] -> sorted: [0.095, 0.250, 0.340]
    known_boxes_.push_back({0.095, 0.250, 0.340, "Small Box"});
    
    // Medium box: [0.255, 0.155, 0.100] -> sorted: [0.100, 0.155, 0.255]  
    known_boxes_.push_back({0.100, 0.155, 0.255, "Medium Box"});
    
    RCLCPP_INFO(this->get_logger(), "Initialized %zu known box types from data folder", known_boxes_.size());
    for (const auto& box : known_boxes_) {
        RCLCPP_INFO(this->get_logger(), "  - %s: %.3f x %.3f x %.3f m (a x b x c)", 
                   box.box_type.c_str(), box.dim_a, box.dim_b, box.dim_c);
    }
    
    // Log the face combinations for reference
    RCLCPP_INFO(this->get_logger(), "Possible face types:");
    for (const auto& box : known_boxes_) {
        RCLCPP_INFO(this->get_logger(), "  %s faces:", box.box_type.c_str());
        RCLCPP_INFO(this->get_logger(), "    - (a,b) face: %.3f x %.3f m, hidden c: %.3f m", 
                   box.dim_a, box.dim_b, box.dim_c);
        RCLCPP_INFO(this->get_logger(), "    - (a,c) face: %.3f x %.3f m, hidden b: %.3f m", 
                   box.dim_a, box.dim_c, box.dim_b);
        RCLCPP_INFO(this->get_logger(), "    - (b,c) face: %.3f x %.3f m, hidden a: %.3f m", 
                   box.dim_b, box.dim_c, box.dim_a);
    }
}

// Estimate box dimensions from point cloud using PCA
PointCloudNode::KnownBoxDimensions PointCloudNode::estimateBoxDimensions(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& box_cloud)
{
    KnownBoxDimensions dims;
    dims.box_type = "Unknown";
    
    if (box_cloud->points.size() < 10) {
        return dims;
    }
    
    // Calculate centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*box_cloud, centroid);
    
    // Calculate covariance matrix for PCA
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrixNormalized(*box_cloud, centroid, covariance_matrix);
    
    // Eigen decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
    Eigen::Matrix3f eigenVectors = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValues = eigen_solver.eigenvalues();
    
    // Get principal axes (sorted by eigenvalue)
    std::vector<std::pair<float, int>> eigen_pairs;
    for (int i = 0; i < 3; ++i) {
        eigen_pairs.push_back(std::make_pair(eigenValues(i), i));
    }
    std::sort(eigen_pairs.begin(), eigen_pairs.end(), std::greater<std::pair<float, int>>());
    
    Eigen::Vector3f x_axis = eigenVectors.col(eigen_pairs[2].second); // Shortest
    Eigen::Vector3f y_axis = eigenVectors.col(eigen_pairs[0].second); // Longest
    Eigen::Vector3f z_axis = eigenVectors.col(eigen_pairs[1].second); // Medium
    
    // Project points onto principal axes to get dimensions
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (const auto& point : box_cloud->points) {
        Eigen::Vector3f pt(point.x - centroid[0], point.y - centroid[1], point.z - centroid[2]);
        float proj_x = pt.dot(x_axis);
        float proj_y = pt.dot(y_axis);
        float proj_z = pt.dot(z_axis);
        
        min_x = std::min(min_x, proj_x); max_x = std::max(max_x, proj_x);
        min_y = std::min(min_y, proj_y); max_y = std::max(max_y, proj_y);
        min_z = std::min(min_z, proj_z); max_z = std::max(max_z, proj_z);
    }
    
    // Calculate dimensions and sort them (a <= b <= c)
    std::vector<double> dimensions = {
        static_cast<double>(max_x - min_x),
        static_cast<double>(max_y - min_y),
        static_cast<double>(max_z - min_z)
    };
    std::sort(dimensions.begin(), dimensions.end());
    
    dims.dim_a = dimensions[0];  // Shortest
    dims.dim_b = dimensions[1];  // Medium
    dims.dim_c = dimensions[2];  // Longest
    
    return dims;
}

// Match detected dimensions to known boxes
std::pair<PointCloudNode::KnownBoxDimensions, double> PointCloudNode::matchToKnownBox(
    const KnownBoxDimensions& detected_dims)
{
    double best_match_score = std::numeric_limits<double>::max();
    KnownBoxDimensions best_match;
    best_match.box_type = "Unknown";
    
    for (const auto& known_box : known_boxes_) {
        // Calculate difference between detected and known dimensions
        double diff_a = std::abs(detected_dims.dim_a - known_box.dim_a);
        double diff_b = std::abs(detected_dims.dim_b - known_box.dim_b);
        double diff_c = std::abs(detected_dims.dim_c - known_box.dim_c);
        
        double total_diff = diff_a + diff_b + diff_c;
        
        if (total_diff < best_match_score) {
            best_match_score = total_diff;
            best_match = known_box;
        }
    }
    
    // Calculate confidence (lower difference = higher confidence)
    // Adjusted tolerance for your box sizes (5cm total error = 0% confidence)
    double confidence = std::max(0.0, 1.0 - (best_match_score / 0.05));
    
    return std::make_pair(best_match, confidence);
}

// Determine face type based on known dimensions
PointCloudNode::DetectedFace PointCloudNode::determineFaceType(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& box_cloud,
    const Eigen::Vector3f& normal_vector,
    const std::string& normal_direction)
{
    DetectedFace face;
    face.normal_direction = normal_direction;
    face.confidence = 0.0;
    
    // Estimate box dimensions from point cloud
    KnownBoxDimensions detected_dims = estimateBoxDimensions(box_cloud);
    
    // Match to known box
    auto match_result = matchToKnownBox(detected_dims);
    KnownBoxDimensions matched_box = match_result.first;
    double match_confidence = match_result.second;
    
    if (match_confidence < 0.5) {
        face.face_type = "Unknown";
        face.confidence = match_confidence;
        return face;
    }
    
    // Calculate which two dimensions are visible based on the viewing direction
    // The dimension parallel to the normal vector is the hidden one
    
    // Get projected 2D dimensions (visible on the face)
    // This is a simplified approach - you might need to refine based on your camera setup
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*box_cloud, min_pt, max_pt);
    
    double visible_dim1 = max_pt.x - min_pt.x;
    double visible_dim2 = max_pt.y - min_pt.y;
    
    // Compare visible dimensions with possible face combinations
    double tolerance = 0.01; // 1cm tolerance for your precise box dimensions
    
    // Check all possible face combinations: (a,b), (a,c), (b,c)
    std::vector<std::tuple<std::string, double, double, double, double>> face_options = {
        {"(a,b)", matched_box.dim_a, matched_box.dim_b, matched_box.dim_c, 
         std::abs(visible_dim1 - matched_box.dim_a) + std::abs(visible_dim2 - matched_box.dim_b)},
        {"(a,c)", matched_box.dim_a, matched_box.dim_c, matched_box.dim_b,
         std::abs(visible_dim1 - matched_box.dim_a) + std::abs(visible_dim2 - matched_box.dim_c)},
        {"(b,c)", matched_box.dim_b, matched_box.dim_c, matched_box.dim_a,
         std::abs(visible_dim1 - matched_box.dim_b) + std::abs(visible_dim2 - matched_box.dim_c)}
    };
    
    // Find best matching face type
    auto best_face = *std::min_element(face_options.begin(), face_options.end(),
        [](const auto& a, const auto& b) { return std::get<4>(a) < std::get<4>(b); });
    
    face.face_type = std::get<0>(best_face);
    face.length1 = std::get<1>(best_face);
    face.length2 = std::get<2>(best_face);
    face.perpendicular_dim = std::get<3>(best_face);
    face.confidence = std::max(0.0, 1.0 - std::get<4>(best_face) / 0.03); // 3cm error = 0% confidence
    
    return face;
}

} // namespace box_detection
