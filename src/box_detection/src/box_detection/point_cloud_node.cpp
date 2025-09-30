#include "box_detection/point_cloud_node.hpp"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <sstream>
#include <chrono>
#include <iomanip>

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
  //pointcloud_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_node/pointcloudfiltered", 10);
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
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = applyStatisticalNoiseRemoval(transformed_cloud);
        
        // Convert to ROS message and publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*transformed_cloud, output_msg);
        output_msg.header = current_header_;
        output_msg.header.frame_id = target_frame_;
    
  
        // sensor_msgs::msg::PointCloud2 output_filtered_msg;
        // pcl::toROSMsg(*filtered_cloud, output_filtered_msg);
        // output_filtered_msg.header = current_header_;
        // output_filtered_msg.header.frame_id = target_frame_;
  

        pointcloud_pub_->publish(output_msg);
        //pointcloud_filtered_pub_->publish(output_filtered_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Published 3D point cloud: %d valid points -> %zu filtered points (%.1f%% kept), frame: %s", 
                            valid_points, transformed_cloud->points.size(), 
                            (transformed_cloud->points.size() * 100.0) / valid_points, target_frame_.c_str());
                            
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in generate3DPointCloud: %s", e.what());
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudNode::applyRANSACNoiseRemoval(
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

        // RANSAC plane segmentation
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(ransac_max_iterations_);
        seg.setDistanceThreshold(ransac_distance_threshold_);
        
        seg.setInputCloud(input_cloud);
        seg.segment(*inliers, *coefficients);
        

        
        // Extract the plane points (these are the filtered/clean points)
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(input_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);  // Keep the plane points (inliers)
        extract.filter(*filtered_cloud);
        
        // Check if we found a reasonable plane
        if (filtered_cloud->empty() || inliers->indices.size() < input_cloud->points.size() * 0.1) {
            RCLCPP_DEBUG(this->get_logger(), "RANSAC found poor plane, returning original cloud");
            return input_cloud;  // Return original if plane detection failed
        }
        
        // Log filtering results
        size_t original_size = input_cloud->points.size();
        size_t filtered_size = filtered_cloud->points.size();
        double removal_percentage = ((double)(original_size - filtered_size) / original_size) * 100.0;
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "RANSAC noise removal: %zu -> %zu points (%.1f%% removed)", 
                    original_size, filtered_size, removal_percentage);
        
        return filtered_cloud;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in RANSAC noise removal: %s", e.what());
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
        
        
        // Convert to ROS message and publish
        sensor_msgs::msg::PointCloud2 box_output_msg;
        pcl::toROSMsg(*combined_box_cloud, box_output_msg);
        box_output_msg.header = current_header_;
        box_output_msg.header.frame_id = target_frame_;
        
        box_pointclouds_pub_->publish(box_output_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "Published box point clouds: %zu total points from %zu boxes, frame: %s", 
                            combined_box_cloud->points.size(), bounding_boxes.size(), target_frame_.c_str());
                            
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

            // **DIRECT FACE DETECTION APPROACH** - Skip RANSAC for face detection
            // Enhanced face detection using known dimensions directly from bounding box
            DetectedFace detected_face = determineFaceType(box_cloud, "FACE");
            
            if (detected_face.confidence < 0.3) {
                RCLCPP_WARN(this->get_logger(), "Box %zu: Low face detection confidence (%.1f%%), skipping", 
                           box_idx + 1, detected_face.confidence * 100.0);
                continue;
            }
            
            // Calculate box centroid for visualization positioning
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*box_cloud, centroid);
            
            // Calculate rough box area from bounding box (for logging purposes)
            pcl::PointXYZRGB min_pt, max_pt;
            pcl::getMinMax3D(*box_cloud, min_pt, max_pt);
            double box_area = (max_pt.x - min_pt.x) * (max_pt.y - min_pt.y);
            
            // Optional: RANSAC plane segmentation for visualization only (plane normal arrows)
            Eigen::Vector3f normal(0, 0, 1); // Default normal pointing up
            bool has_plane_normal = false;
            
            // if (true) { // Set to false to completely skip RANSAC
            //     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            //     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            //     pcl::SACSegmentation<pcl::PointXYZRGB> seg;
                
            //     seg.setOptimizeCoefficients(true);
            //     seg.setModelType(pcl::SACMODEL_PLANE);
            //     seg.setMethodType(pcl::SAC_RANSAC);
            //     seg.setMaxIterations(ransac_max_iterations_);
            //     seg.setDistanceThreshold(ransac_distance_threshold_);
                
            //     seg.setInputCloud(filtered_cloud);
            //     seg.segment(*inliers, *coefficients);
                
            //     if (!inliers->indices.empty() && inliers->indices.size() > filtered_cloud->points.size() * 0.3) {
            //         // Good plane found - use its normal for visualization
            //         normal = Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            //         normal.normalize();
            //         has_plane_normal = true;
                    
            //         RCLCPP_DEBUG(this->get_logger(), "Box %zu: Found plane with %.1f%% inliers for normal visualization", 
            //                    box_idx + 1, (double)inliers->indices.size() / filtered_cloud->points.size() * 100.0);
            //     }
            // }
            
            RCLCPP_INFO(this->get_logger(), 
                       "Box %zu: Direct face detection - %.3fx%.3f m bounding box (area: %.4f m²)", 
                       box_idx + 1, max_pt.x - min_pt.x, max_pt.y - min_pt.y, box_area);
                       
            RCLCPP_INFO(this->get_logger(),
                       "  🎯 Face type: %s (confidence: %.1f%%), visible dims: %.3fx%.3f m, hidden dim: %.3f m",
                       detected_face.face_type.c_str(), detected_face.confidence * 100.0,
                       detected_face.length1, detected_face.length2, detected_face.perpendicular_dim);
            
            // Add enhanced information to summary string
            if (!face_detection_summary.empty()) {
                face_detection_summary += ";";
            }
            face_detection_summary += "Box" + std::to_string(box_idx + 1) + 
                                    ":face_type:" + detected_face.face_type +
                                    ":confidence:" + std::to_string(detected_face.confidence) +
                                    ":visible_dims:" + std::to_string(detected_face.length1) + "x" + std::to_string(detected_face.length2) +
                                    ":hidden_dim:" + std::to_string(detected_face.perpendicular_dim) +
                                    ":area:" + std::to_string(box_area) +
                                    ":normal:" + std::to_string(normal.x()) + "," + std::to_string(normal.y()) + "," + std::to_string(normal.z());
            
            // Create visualization marker for the face with enhanced info
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = target_frame_;
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "box_faces";
            marker.id = box_idx;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2] + 0.05; // Slightly above the centroid
            marker.pose.orientation.w = 1.0;
            
            marker.scale.z = 0.025; // Text size
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            
            marker.text = "Box" + std::to_string(box_idx + 1) + ": " + detected_face.face_type;
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            
            marker_array.markers.push_back(marker);
            
            // Create arrow marker for normal vector (only if RANSAC found a good plane)
            if (has_plane_normal) {
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
            }
            
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
            // // Apply noise removal first - use the filtered point cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = applyRANSACNoiseRemoval(box_cloud);
            
            if (filtered_cloud->points.size() < 30) {
                RCLCPP_WARN(this->get_logger(), "Box %zu: Too few points (%zu) for reliable pose estimation", 
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
            
            seg.setInputCloud(box_cloud);
            seg.segment(*inliers, *coefficients);
            
            if (inliers->indices.empty()) {
                RCLCPP_WARN(this->get_logger(), "Box %zu: No plane found", box_idx + 1);
                continue;
            }
            
            // Extract the plane
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(box_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*plane_cloud);
            
            if (plane_cloud->points.empty()) {
                continue;
            }





            // **STEP 1: GET FACE DETECTION INFORMATION**
            DetectedFace detected_face = determineFaceType(filtered_cloud, "FACE");
            
            if (detected_face.confidence < 0.5) {
                RCLCPP_WARN(this->get_logger(), "Box %zu: Low face detection confidence (%.1f%%), using basic PCA", 
                           box_idx + 1, detected_face.confidence * 100.0);
            }
            
            // **STEP 2: CALCULATE ENHANCED TRANSLATION**
            // Use filtered point cloud centroid for better accuracy
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*filtered_cloud, centroid);
            
            // **STEP 3: CALCULATE ENHANCED ROTATION USING FACE INFORMATION**
            Eigen::Matrix3f rotation_matrix;
            Eigen::Vector3f estimated_dimensions;
            
            if (detected_face.confidence > 0.5) {
                // Use face detection to create constrained coordinate frame
                rotation_matrix = estimateOrientationWithFaceConstraints(filtered_cloud, detected_face);
                
                // Use known dimensions from face detection
                std::string face_type = detected_face.face_type;
                if (face_type.find("Small_Box") != std::string::npos) {
                    estimated_dimensions = Eigen::Vector3f(0.095f, 0.250f, 0.340f); // a, b, c
                } else if (face_type.find("Medium_Box") != std::string::npos) {
                    estimated_dimensions = Eigen::Vector3f(0.100f, 0.155f, 0.255f); // a, b, c
                } else {
                    // Fallback to measured dimensions
                    estimated_dimensions = measureBoxDimensions(filtered_cloud, rotation_matrix);
                }
                
                RCLCPP_INFO(this->get_logger(), 
                           "Box %zu: Using face-constrained pose (face: %s, confidence: %.1f%%)", 
                           box_idx + 1, detected_face.face_type.c_str(), detected_face.confidence * 100.0);
            } else {
                // Fallback to basic PCA approach
                rotation_matrix = estimateOrientationWithPCA(filtered_cloud);
                estimated_dimensions = measureBoxDimensions(filtered_cloud, rotation_matrix);
                
                RCLCPP_INFO(this->get_logger(), 
                           "Box %zu: Using PCA-based pose (face detection failed)", box_idx + 1);
            }
            
            // **STEP 4: REFINE TRANSLATION USING BOX GEOMETRY**
            // Adjust centroid to box geometric center using known dimensions
            Eigen::Vector3f refined_centroid = refineCentroidWithGeometry(
                filtered_cloud, 
                Eigen::Vector3f(centroid[0], centroid[1], centroid[2]), 
                rotation_matrix, 
                estimated_dimensions
            );
            
            // Store pose data for image visualization
            box_centroids.push_back(refined_centroid);
            box_rotations.push_back(rotation_matrix);
            
            // Convert to quaternion
            Eigen::Quaternionf quaternion(rotation_matrix);
            quaternion.normalize();
            
            // **STEP 5: LOG ENHANCED POSE INFORMATION**
            RCLCPP_INFO(this->get_logger(), 
                       "📦 Box %zu Enhanced 6D Pose:", box_idx + 1);
            RCLCPP_INFO(this->get_logger(), 
                       "   Translation: [%.3f, %.3f, %.3f] m", 
                       refined_centroid[0], refined_centroid[1], refined_centroid[2]);
            RCLCPP_INFO(this->get_logger(), 
                       "   Rotation (quat): [%.3f, %.3f, %.3f, %.3f]", 
                       quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
            RCLCPP_INFO(this->get_logger(), 
                       "   Estimated Dims: %.3fx%.3fx%.3f m", 
                       estimated_dimensions[0], estimated_dimensions[1], estimated_dimensions[2]);
            RCLCPP_INFO(this->get_logger(), 
                       "   Face Info: %s (conf: %.1f%%)", 
                       detected_face.face_type.c_str(), detected_face.confidence * 100.0);
            
            // Build enhanced pose data string
            if (!pose_data_summary.empty()) {
                pose_data_summary += ";";
            }
            pose_data_summary += "Box" + std::to_string(box_idx + 1) + 
                               ":pos:" + std::to_string(refined_centroid[0]) + "," + 
                               std::to_string(refined_centroid[1]) + "," + std::to_string(refined_centroid[2]) +
                               ":quat:" + std::to_string(quaternion.x()) + "," + std::to_string(quaternion.y()) + "," + 
                               std::to_string(quaternion.z()) + "," + std::to_string(quaternion.w()) +
                               ":dim:" + std::to_string(estimated_dimensions[0]) + "," + 
                               std::to_string(estimated_dimensions[1]) + "," + std::to_string(estimated_dimensions[2]) +
                               ":face:" + detected_face.face_type +
                               ":confidence:" + std::to_string(detected_face.confidence);
            
            // Create coordinate frame visualization
            // Get axes from rotation matrix
            Eigen::Vector3f x_axis = rotation_matrix.col(0);
            Eigen::Vector3f y_axis = rotation_matrix.col(1);
            Eigen::Vector3f z_axis = rotation_matrix.col(2);
            
            // X-axis (red)
            visualization_msgs::msg::Marker x_marker;
            x_marker.header.frame_id = target_frame_;
            x_marker.header.stamp = this->get_clock()->now();
            x_marker.ns = "box_coordinate_frames";
            x_marker.id = box_idx * 3;
            x_marker.type = visualization_msgs::msg::Marker::ARROW;
            x_marker.action = visualization_msgs::msg::Marker::ADD;
            
            x_marker.pose.position.x = refined_centroid[0];
            x_marker.pose.position.y = refined_centroid[1];
            x_marker.pose.position.z = refined_centroid[2];
            
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
        
        // TEMPORARY: Save annotated image to disk
        saveAnnotatedImage(box_centroids, box_rotations, "box_pose_detection");
        
        // TEMPORARY: Save RGB image with 3D coordinates to /home/oguz/neura_tasks_ws
        saveImageWithCoordinates(box_centroids, box_rotations, "pose_coordinates");
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
    known_boxes_.push_back({0.095, 0.250, 0.340, "Small_Box"});
    
    // Medium box: [0.255, 0.155, 0.100] -> sorted: [0.100, 0.155, 0.255]  
    known_boxes_.push_back({0.100, 0.155, 0.255, "Medium_Box"});
    
    RCLCPP_INFO(this->get_logger(), "Initialized %zu known box types using bounding box approach", known_boxes_.size());
    for (const auto& box : known_boxes_) {
        RCLCPP_INFO(this->get_logger(), "  - %s: %.3f x %.3f x %.3f m (a x b x c)", 
                   box.box_type.c_str(), box.dim_a, box.dim_b, box.dim_c);
    }
    
    // Log the face combinations that can be detected
    RCLCPP_INFO(this->get_logger(), "Detectable face types (with occlusion handling):");
    for (const auto& box : known_boxes_) {
        RCLCPP_INFO(this->get_logger(), "  %s faces:", box.box_type.c_str());
        RCLCPP_INFO(this->get_logger(), "    - %s_(a,b): %.3f x %.3f m face, %.3f m hidden", 
                   box.box_type.c_str(), box.dim_a, box.dim_b, box.dim_c);
        RCLCPP_INFO(this->get_logger(), "    - %s_(a,c): %.3f x %.3f m face, %.3f m hidden", 
                   box.box_type.c_str(), box.dim_a, box.dim_c, box.dim_b);
        RCLCPP_INFO(this->get_logger(), "    - %s_(b,c): %.3f x %.3f m face, %.3f m hidden", 
                   box.box_type.c_str(), box.dim_b, box.dim_c, box.dim_a);
    }
    
    RCLCPP_INFO(this->get_logger(), "🔍 Occlusion handling: If visible dimension < expected - 5cm, treated as occluded");
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

// Determine face type based on bounding box real-world dimensions
PointCloudNode::DetectedFace PointCloudNode::determineFaceType(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& box_cloud,
    const std::string& normal_direction)
{
    DetectedFace face;
    face.normal_direction = normal_direction;
    face.confidence = 0.0;
    face.face_type = "Unknown";
    
    if (box_cloud->points.empty()) {
        return face;
    }
    
    // Calculate real-world dimensions from the bounding box
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*box_cloud, min_pt, max_pt);
    
    // Get the three dimensions of the bounding box
    double dim_x = std::abs(max_pt.x - min_pt.x);
    double dim_y = std::abs(max_pt.y - min_pt.y);
    double dim_z = std::abs(max_pt.z - min_pt.z);
    
    // Store dimensions and sort them to find the two largest (visible dimensions)
    std::vector<std::pair<double, char>> dimensions = {
        {dim_x, 'X'}, {dim_y, 'Y'}, {dim_z, 'Z'}
    };
    std::sort(dimensions.begin(), dimensions.end(), std::greater<std::pair<double, char>>());
    
    double visible_dim1 = dimensions[0].first;  // Largest visible dimension
    double visible_dim2 = dimensions[1].first;  // Second largest visible dimension
    double hidden_dim = dimensions[2].first;    // Smallest dimension (might be truncated)
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Bounding box dims: X=%.3f, Y=%.3f, Z=%.3f -> Visible: %.3fx%.3f, Hidden: %.3f",
                dim_x, dim_y, dim_z, visible_dim1, visible_dim2, hidden_dim);
    
    // Try to match with known boxes and find best face type
    double best_confidence = 0.0;
    DetectedFace best_face;
    
    for (const auto& known_box : known_boxes_) {
        // Check all possible face combinations for this box type
        std::vector<std::tuple<std::string, double, double, double>> face_combinations = {
            {"(a,b)", known_box.dim_a, known_box.dim_b, known_box.dim_c},  // Face showing a×b, hidden c
            {"(a,c)", known_box.dim_a, known_box.dim_c, known_box.dim_b},  // Face showing a×c, hidden b  
            {"(b,c)", known_box.dim_b, known_box.dim_c, known_box.dim_a}   // Face showing b×c, hidden a
        };
        
        for (const auto& face_combo : face_combinations) {
            std::string face_name = std::get<0>(face_combo);
            double expected_dim1 = std::max(std::get<1>(face_combo), std::get<2>(face_combo));  // Larger face dimension
            double expected_dim2 = std::min(std::get<1>(face_combo), std::get<2>(face_combo));  // Smaller face dimension
            double expected_hidden = std::get<3>(face_combo);  // Hidden dimension
            
            // Calculate errors for visible dimensions
            double error_dim1 = std::abs(visible_dim1 - expected_dim1);
            double error_dim2 = std::abs(visible_dim2 - expected_dim2);
            
            // Handle occlusion for visible dimensions
            // If a visible dimension is significantly smaller than expected, it might be occluded
            double occlusion_tolerance = 0.05; // 5cm tolerance for occlusion
            bool dim1_occluded = (visible_dim1 < expected_dim1 - occlusion_tolerance);
            bool dim2_occluded = (visible_dim2 < expected_dim2 - occlusion_tolerance);
            
            // Adjust errors if occlusion is detected
            if (dim1_occluded) {
                error_dim1 = 0.0; // Don't penalize if occluded
                RCLCPP_DEBUG(this->get_logger(), "Dimension 1 occlusion detected: visible=%.3f < expected=%.3f", 
                           visible_dim1, expected_dim1);
            }
            if (dim2_occluded) {
                error_dim2 = 0.0; // Don't penalize if occluded
                RCLCPP_DEBUG(this->get_logger(), "Dimension 2 occlusion detected: visible=%.3f < expected=%.3f", 
                           visible_dim2, expected_dim2);
            }
            
            // Calculate total error
            double total_error = error_dim1 + error_dim2;
            
            // Calculate confidence (lower error = higher confidence)
            double confidence = std::max(0.0, 1.0 - (total_error / 0.08)); // 8cm total error = 0% confidence
            
            // Bonus confidence if dimensions match well
            double dimension_tolerance = 0.02; // 2cm exact match tolerance
            if (error_dim1 < dimension_tolerance && error_dim2 < dimension_tolerance) {
                confidence += 0.2; // 20% bonus for exact matches
            }
            
            // Apply occlusion penalty (but still allow detection)
            if (dim1_occluded || dim2_occluded) {
                confidence *= 0.8; // 20% penalty for occlusion, but still detectable
            }
            
            confidence = std::min(1.0, confidence); // Cap at 100%
            
            RCLCPP_DEBUG(this->get_logger(), 
                        "Testing %s %s: expected %.3fx%.3f, errors %.3f+%.3f=%.3f, confidence %.3f",
                        known_box.box_type.c_str(), face_name.c_str(), 
                        expected_dim1, expected_dim2, error_dim1, error_dim2, total_error, confidence);
            
            // Update best match if this is better
            if (confidence > best_confidence) {
                best_confidence = confidence;
                best_face.face_type = known_box.box_type + "_" + face_name;
                best_face.length1 = expected_dim1;
                best_face.length2 = expected_dim2; 
                best_face.perpendicular_dim = expected_hidden;
                best_face.confidence = confidence;
                best_face.normal_direction = normal_direction;
            }
        }
    }
    
    // Return best match if confidence is reasonable
    if (best_confidence > 0.3) { // Minimum 30% confidence threshold
        RCLCPP_DEBUG(this->get_logger(), 
                    "Best match: %s (%.1f%% confidence), visible: %.3fx%.3f m, hidden: %.3f m",
                    best_face.face_type.c_str(), best_confidence * 100.0,
                    best_face.length1, best_face.length2, best_face.perpendicular_dim);
        return best_face;
    }
    
    // If no good match found, return unknown
    face.face_type = "Unknown";
    face.confidence = 0.0;
    return face;
}

// Helper function: Estimate orientation using face detection constraints
Eigen::Matrix3f PointCloudNode::estimateOrientationWithFaceConstraints(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
    const DetectedFace& detected_face)
{
    // Start with PCA as baseline
    Eigen::Matrix3f pca_rotation = estimateOrientationWithPCA(filtered_cloud);
    
    if (detected_face.confidence < 0.5) {
        return pca_rotation; // Fallback to PCA if face detection is unreliable
    }
    
    // Use face detection to constrain the orientation
    // The key insight: we know which face we're looking at, so we can align axes accordingly
    
    std::string face_type = detected_face.face_type;
    Eigen::Matrix3f constrained_rotation = pca_rotation;
    
    // Extract box type and face combination
    bool is_small_box = face_type.find("Small_Box") != std::string::npos;
    bool is_ab_face = face_type.find("(a,b)") != std::string::npos;
    bool is_ac_face = face_type.find("(a,c)") != std::string::npos;
    bool is_bc_face = face_type.find("(b,c)") != std::string::npos;
    
    // Get expected dimensions for this box type
    float dim_a = is_small_box ? 0.095f : 0.100f;
    float dim_b = is_small_box ? 0.250f : 0.155f; 
    float dim_c = is_small_box ? 0.340f : 0.255f;
    
    // Measure actual dimensions from point cloud
    Eigen::Vector3f measured_dims = measureBoxDimensions(filtered_cloud, pca_rotation);
    std::sort(measured_dims.data(), measured_dims.data() + 3);
    
    // Create constrained rotation matrix based on face type
    if (is_ab_face) {
        // Face shows a×b, hidden dimension is c
        // Ensure the largest measured dimension aligns with the hidden axis (c)
        // and the two smaller visible dimensions align with a and b
        constrained_rotation = alignAxesWithDimensions(pca_rotation, measured_dims, 
                                                     Eigen::Vector3f(dim_a, dim_b, dim_c));
    } else if (is_ac_face) {
        // Face shows a×c, hidden dimension is b
        constrained_rotation = alignAxesWithDimensions(pca_rotation, measured_dims,
                                                     Eigen::Vector3f(dim_a, dim_c, dim_b));
    } else if (is_bc_face) {
        // Face shows b×c, hidden dimension is a
        constrained_rotation = alignAxesWithDimensions(pca_rotation, measured_dims,
                                                     Eigen::Vector3f(dim_b, dim_c, dim_a));
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Face-constrained rotation for %s: improved orientation based on expected dims",
                face_type.c_str());
    
    return constrained_rotation;
}

// Helper function: Estimate orientation using basic PCA
Eigen::Matrix3f PointCloudNode::estimateOrientationWithPCA(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud)
{
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
    
    // Create rotation matrix: largest eigenvalue -> Z, medium -> Y, smallest -> X
    Eigen::Vector3f x_axis = eigenVectors.col(eigen_pairs[2].second); // Shortest axis (X)
    Eigen::Vector3f y_axis = eigenVectors.col(eigen_pairs[1].second); // Medium axis (Y) 
    Eigen::Vector3f z_axis = eigenVectors.col(eigen_pairs[0].second); // Longest axis (Z)
    
    // Ensure right-handed coordinate system
    if (x_axis.cross(y_axis).dot(z_axis) < 0) {
        z_axis = -z_axis;
    }
    
    // Create rotation matrix
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;
    
    return rotation_matrix;
}

// Helper function: Measure box dimensions along given axes
Eigen::Vector3f PointCloudNode::measureBoxDimensions(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
    const Eigen::Matrix3f& rotation_matrix)
{
    // Calculate centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*filtered_cloud, centroid);
    
    // Get axes from rotation matrix
    Eigen::Vector3f x_axis = rotation_matrix.col(0);
    Eigen::Vector3f y_axis = rotation_matrix.col(1);
    Eigen::Vector3f z_axis = rotation_matrix.col(2);
    
    // Project points onto axes and find dimensions
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
        
        min_x = std::min(min_x, proj_x); max_x = std::max(max_x, proj_x);
        min_y = std::min(min_y, proj_y); max_y = std::max(max_y, proj_y);
        min_z = std::min(min_z, proj_z); max_z = std::max(max_z, proj_z);
    }
    
    return Eigen::Vector3f(max_x - min_x, max_y - min_y, max_z - min_z);
}

// Helper function: Align axes with expected dimensions
Eigen::Matrix3f PointCloudNode::alignAxesWithDimensions(
    const Eigen::Matrix3f& pca_rotation,
    const Eigen::Vector3f& measured_dims,
    const Eigen::Vector3f& expected_dims)
{
    // This is a simplified alignment - in practice, you might want more sophisticated matching
    // For now, we use the PCA rotation as-is but could add dimension-based corrections here
    
    // Check if measured dimensions roughly match expected (within 20% tolerance)
    float tolerance = 0.2f;
    bool dims_match = true;
    
    for (int i = 0; i < 3; ++i) {
        float error = std::abs(measured_dims[i] - expected_dims[i]) / expected_dims[i];
        if (error > tolerance) {
            dims_match = false;
            break;
        }
    }
    
    if (dims_match) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Measured dimensions match expected, using PCA rotation");
        return pca_rotation;
    } else {
        // Could implement more sophisticated axis swapping/flipping here
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Dimension mismatch detected, keeping PCA rotation for now");
        return pca_rotation;
    }
}

// Helper function: Refine centroid using box geometry
Eigen::Vector3f PointCloudNode::refineCentroidWithGeometry(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
    const Eigen::Vector3f& initial_centroid,
    const Eigen::Matrix3f& rotation_matrix,
    [[maybe_unused]] const Eigen::Vector3f& expected_dimensions)
{
    // Calculate the geometric center by finding the bounding box center along each axis
    Eigen::Vector3f x_axis = rotation_matrix.col(0);
    Eigen::Vector3f y_axis = rotation_matrix.col(1);
    Eigen::Vector3f z_axis = rotation_matrix.col(2);
    
    // Project points and find min/max along each axis
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (const auto& point : filtered_cloud->points) {
        Eigen::Vector3f pt(point.x - initial_centroid[0], 
                          point.y - initial_centroid[1], 
                          point.z - initial_centroid[2]);
        float proj_x = pt.dot(x_axis);
        float proj_y = pt.dot(y_axis);
        float proj_z = pt.dot(z_axis);
        
        min_x = std::min(min_x, proj_x); max_x = std::max(max_x, proj_x);
        min_y = std::min(min_y, proj_y); max_y = std::max(max_y, proj_y);
        min_z = std::min(min_z, proj_z); max_z = std::max(max_z, proj_z);
    }
    
    // Calculate geometric center offsets
    float center_offset_x = (max_x + min_x) / 2.0f;
    float center_offset_y = (max_y + min_y) / 2.0f;
    float center_offset_z = (max_z + min_z) / 2.0f;
    
    // Apply offsets to get refined centroid
    Eigen::Vector3f refined_centroid = initial_centroid + 
                                     center_offset_x * x_axis +
                                     center_offset_y * y_axis +
                                     center_offset_z * z_axis;
    
    // Log refinement
    float refinement_distance = (refined_centroid - initial_centroid).norm();
    RCLCPP_DEBUG(this->get_logger(), 
                "Centroid refined by %.3f m using box geometry", refinement_distance);
    
    return refined_centroid;
}

// Temporary function: Save RGB image with coordinate annotations
void PointCloudNode::saveAnnotatedImage(
    const std::vector<Eigen::Vector3f>& centroids,
    const std::vector<Eigen::Matrix3f>& rotations,
    const std::string& filename_prefix)
{
    if (current_color_image_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No color image available for saving annotated image");
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
            RCLCPP_WARN(this->get_logger(), "Failed to get transform for image annotation: %s", ex.what());
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
        
        // Clone the original image for annotation
        cv::Mat annotated_image = current_color_image_.clone();
        
        // Define colors and settings
        cv::Scalar x_color(0, 0, 255);     // Red for X-axis
        cv::Scalar y_color(0, 255, 0);     // Green for Y-axis  
        cv::Scalar z_color(255, 0, 0);     // Blue for Z-axis
        cv::Scalar center_color(255, 255, 255);  // White for center
        cv::Scalar text_color(255, 255, 255);    // White for text
        cv::Scalar text_outline(0, 0, 0);        // Black outline for text
        
        float axis_length = 0.08f; // 8cm axis length for better visibility
        int line_thickness = 4;
        
        // Add title and timestamp
        std::time_t now = std::time(0);
        std::string timestamp = std::ctime(&now);
        timestamp.pop_back(); // Remove newline
        
        // Draw clean coordinate systems for each detected box (no text labels)
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
            auto isPointValid = [&annotated_image](const cv::Point& pt) -> bool {
                return pt.x >= 0 && pt.y >= 0 && pt.x < annotated_image.cols && pt.y < annotated_image.rows;
            };
            
            if (!isPointValid(center_2d)) {
                continue; // Skip if center is outside image
            }
            
            // Draw coordinate axes (no text labels)
            if (isPointValid(x_end_2d)) {
                cv::arrowedLine(annotated_image, center_2d, x_end_2d, x_color, line_thickness, 8, 0, 0.3);
            }
            
            if (isPointValid(y_end_2d)) {
                cv::arrowedLine(annotated_image, center_2d, y_end_2d, y_color, line_thickness, 8, 0, 0.3);
            }
            
            if (isPointValid(z_end_2d)) {
                cv::arrowedLine(annotated_image, center_2d, z_end_2d, z_color, line_thickness, 8, 0, 0.3);
            }
            
            // Draw box center point
            cv::circle(annotated_image, center_2d, 8, center_color, -1);
            cv::circle(annotated_image, center_2d, 10, cv::Scalar(0, 0, 0), 3);
        }
        
        // Generate filename with timestamp
        auto now_time = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now_time);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        
        std::string filename = "/home/oguz/" + filename_prefix + "_" + ss.str() + ".jpg";
        
        // Save the annotated image
        bool success = cv::imwrite(filename, annotated_image);
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), 
                       "💾 Saved annotated image with %zu coordinate systems to: %s", 
                       centroids.size(), filename.c_str());
            RCLCPP_INFO(this->get_logger(), 
                       "   Image size: %dx%d, Boxes detected: %zu", 
                       annotated_image.cols, annotated_image.rows, centroids.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save annotated image to: %s", filename.c_str());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in saveAnnotatedImage: %s", e.what());
    }
}

// Temporary function: Save RGB image with detected coordinates to disk
void PointCloudNode::saveImageWithCoordinates(
    const std::vector<Eigen::Vector3f>& centroids,
    const std::vector<Eigen::Matrix3f>& rotations,
    const std::string& filename_prefix)
{
    if (current_color_image_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No color image available for saving with coordinates");
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
            RCLCPP_WARN(this->get_logger(), 
                       "Failed to get transform for image saving: %s", ex.what());
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
        cv::Mat saved_image = current_color_image_.clone();
        
        // Define axis colors (BGR format for OpenCV)
        cv::Scalar x_color(0, 0, 255);    // Red for X-axis
        cv::Scalar y_color(0, 255, 0);    // Green for Y-axis  
        cv::Scalar z_color(255, 0, 0);    // Blue for Z-axis
        cv::Scalar center_color(255, 255, 255); // White for center
        
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
            auto isPointValid = [&saved_image](const cv::Point& pt) -> bool {
                return pt.x >= 0 && pt.y >= 0 && pt.x < saved_image.cols && pt.y < saved_image.rows;
            };
            
            if (!isPointValid(center_2d)) {
                continue; // Skip if center is outside image
            }
            
            // Draw coordinate axes (no text labels)
            if (isPointValid(x_end_2d)) {
                cv::arrowedLine(saved_image, center_2d, x_end_2d, x_color, line_thickness, 8, 0, 0.3);
            }
            
            if (isPointValid(y_end_2d)) {
                cv::arrowedLine(saved_image, center_2d, y_end_2d, y_color, line_thickness, 8, 0, 0.3);
            }
            
            if (isPointValid(z_end_2d)) {
                cv::arrowedLine(saved_image, center_2d, z_end_2d, z_color, line_thickness, 8, 0, 0.3);
            }
            
            // Draw box center point
            cv::circle(saved_image, center_2d, 8, center_color, -1);
            cv::circle(saved_image, center_2d, 9, cv::Scalar(0, 0, 0), 2);
        }
        
        // Generate filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
        
        std::string filename = "/home/oguz/neura_tasks_ws/" + filename_prefix + "_" + ss.str() + ".jpg";
        
        // Save the image
        bool success = cv::imwrite(filename, saved_image);
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), 
                       "✅ Saved pose detection image: %s", filename.c_str());
            RCLCPP_INFO(this->get_logger(), 
                       "   📍 Detected %zu boxes with coordinates and axes", centroids.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                        "❌ Failed to save image: %s", filename.c_str());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in saveImageWithCoordinates: %s", e.what());
    }
}

} // namespace box_detection
