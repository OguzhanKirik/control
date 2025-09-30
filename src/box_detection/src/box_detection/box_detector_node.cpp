// Restored full node with image + depth subscriptions and point cloud generation
#include "box_detection/box_detector_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/string.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
// TF and PCL transform includes moved to point_cloud_node
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <pcl/common/transforms.h>
// #include <Eigen/Dense>
// PCL filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <visualization_msgs/msg/marker_array.hpp>
// TEMPORARY: Includes for image saving functionality
#include <chrono>
#include <iomanip>
#include <sstream>

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

  // Publishers for 2D detection
  boundingBox_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/box_detection/boxes_image", 10);
  boundingBox_coordinates_pub_ = this->create_publisher<std_msgs::msg::String>("/box_detection/boxes_coordinates", 10);

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
    
    // Trigger detection when both images are available
    if (has_depth_image_ && has_camera_info_) {
      detect2DBoxes();
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
    
    // Trigger detection when both images are available
    if (has_color_image_ && has_camera_info_) {
      detect2DBoxes();
    }

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
    // Check if we have all required data
    if (!has_color_image_ || !has_depth_image_ || !has_camera_info_) {
        return;  // Exit early if not all data is available
    }
    
    if (current_color_image_.empty() || current_depth_image_.empty()) {
        return;
    }

    // Create working copies of the images
    cv::Mat color_image = current_color_image_.clone();
    cv::Mat depth_image = current_depth_image_.clone();
    
    // Normalize depth image for edge detection
    cv::Mat depth_normalized;
    depth_image.convertTo(depth_normalized, CV_8UC1, 255.0/65535.0);

    // 1. DEPTH-BASED EDGE DETECTION
    cv::Mat depth_edges;
    cv::Canny(depth_normalized, depth_edges, 4, 20);
    
    // 2. COLOR-BASED EDGE DETECTION
    cv::Mat gray;
    cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat color_edges;
    cv::Canny(gray, color_edges, 73, 80);

    // 3. COMBINE COLOR AND DEPTH EDGES
    cv::Mat combined_edges;
    cv::bitwise_or(color_edges, depth_edges, combined_edges);
    
    // 3b. MORPHOLOGICAL CLOSING - Connect nearby edges while preserving shapes
    cv::Mat closed_edges;
    cv::Mat morphology_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::morphologyEx(combined_edges, closed_edges, cv::MORPH_CLOSE, morphology_kernel, cv::Point(-1, -1), 1);

    // 3c. ADDITIONAL DILATION - Make lines thicker for better contour detection
    cv::Mat thickened_edges;
    cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(closed_edges, thickened_edges, dilate_kernel, cv::Point(-1, -1), 2);
    
    // 4. FIND CONTOURS ON PROCESSED EDGES
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thickened_edges, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    
    // 5. ANALYZE CONTOURS AND CREATE BOUNDING BOXES
    std::vector<cv::Rect> bounding_boxes;
    cv::Mat bbox_image = color_image.clone();
    std::string coordinates_str = "";
    
    for (size_t i = 0; i < contours.size(); i++) {
        // Filter contours by area to remove noise
        double area = cv::contourArea(contours[i]);
        if (area < 50 || area > 100000) {
            continue;
        }
        
        // Get bounding rectangle
        cv::Rect bbox = cv::boundingRect(contours[i]);
        
        // Filter by size to get reasonable bounding boxes
        if (bbox.width < 45 || bbox.height < 45) {
            continue;
        }
        if (bbox.width > 200 || bbox.height > 200) {
            continue;
        }
        
        // Basic aspect ratio check
        double aspect_ratio = (double)bbox.width / bbox.height;
        if (aspect_ratio < 0.05 || aspect_ratio > 20.0) {
            continue;
        }
        
        bounding_boxes.push_back(bbox);
        
        // Draw bounding box with different colors
        cv::Scalar box_color;
        switch (bounding_boxes.size() % 4) {
            case 0: box_color = cv::Scalar(0, 255, 0); break;    // Green
            case 1: box_color = cv::Scalar(255, 0, 0); break;    // Blue
            case 2: box_color = cv::Scalar(0, 0, 255); break;    // Red
            case 3: box_color = cv::Scalar(255, 255, 0); break;  // Cyan
        }
        
        cv::rectangle(bbox_image, bbox, box_color, 2);
        
        // Add bounding box number and size info
        std::string label = "Box" + std::to_string(bounding_boxes.size());
        cv::Point label_pos(bbox.x + 5, bbox.y + 20);
        cv::putText(bbox_image, label, label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2);
        
        // Add size information
        std::string size_info = std::to_string(bbox.width) + "x" + std::to_string(bbox.height);
        cv::Point size_pos(bbox.x + 5, bbox.y + bbox.height - 5);
        cv::putText(bbox_image, size_info, size_pos, cv::FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1);
        
        // Build coordinates string
        if (!coordinates_str.empty()) {
            coordinates_str += ";";
        }
        coordinates_str += std::to_string(bbox.x) + "," + std::to_string(bbox.y) + "," + 
                          std::to_string(bbox.width) + "," + std::to_string(bbox.height);
    }
    
    // 6. PUBLISH RESULTS
    // Publish annotated image
    auto img_msg = cv_bridge::CvImage(current_header_, sensor_msgs::image_encodings::BGR8, bbox_image).toImageMsg();
    boundingBox_image_pub_->publish(*img_msg);
    
    // TEMPORARY: Save 2D detection image to local computer - COMMENTED OUT
    // saveDetectionImage(bbox_image, "2d_box_detection");
    
    // Publish bounding box coordinates
    auto coord_msg = std_msgs::msg::String();
    coord_msg.data = coordinates_str;
    boundingBox_coordinates_pub_->publish(coord_msg);
    
    // Log detection results
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Detected %zu boxes from %zu contours", bounding_boxes.size(), contours.size());
}

/*
// TEMPORARY: Save 2D box detection image to local computer - COMMENTED OUT
void BoxDetector::saveDetectionImage(const cv::Mat& annotated_image, const std::string& filename_prefix)
{
    if (annotated_image.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot save empty detection image");
        return;
    }
    
    try {
        // Generate filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << "/home/oguz/neura_tasks_ws/" << filename_prefix << "_" 
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") 
           << "_" << std::setfill('0') << std::setw(3) << ms.count() << ".png";
        
        std::string filename = ss.str();
        
        // Save the image
        bool success = cv::imwrite(filename, annotated_image);
        
        if (success) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "📸 Saved 2D detection image: %s (size: %dx%d)", 
                               filename.c_str(), annotated_image.cols, annotated_image.rows);
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to save 2D detection image: %s", filename.c_str());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in saveDetectionImage: %s", e.what());
    }
}
*/

} // namespace box_detection

