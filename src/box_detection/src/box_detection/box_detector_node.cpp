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
#include <pcl/search/kdtree.h>

namespace box_detection {

BoxDetector::BoxDetector() : Node("box_detector_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting Box Detector Node");

  has_color_image_ = false;
  has_depth_image_ = false;
  has_camera_info_ = false;
  first_image_logged_ = false;

  fx_ = 525.0; fy_ = 525.0; cx_ = 320.0; cy_ = 240.0;

  // TF parameters disabled
  // this->declare_parameter("world_frame", world_frame_);
  // this->declare_parameter("camera_optical_frame", camera_optical_frame_);
  // this->declare_parameter("floor_margin_m", floor_margin_m_);
  this->declare_parameter("sync_tolerance_ms", sync_tolerance_ms_);
  this->declare_parameter("min_height_m", min_height_m_);
  this->declare_parameter("cluster_tolerance_m", cluster_tolerance_m_);
  this->declare_parameter("cluster_min_size", cluster_min_size_);
  this->declare_parameter("cluster_max_size", cluster_max_size_);
  // world_frame_ = this->get_parameter("world_frame").as_string();
  // camera_optical_frame_ = this->get_parameter("camera_optical_frame").as_string();
  // floor_margin_m_ = this->get_parameter("floor_margin_m").as_double();
  sync_tolerance_ms_ = this->get_parameter("sync_tolerance_ms").as_int();
  min_height_m_ = this->get_parameter("min_height_m").as_double();
  cluster_tolerance_m_ = this->get_parameter("cluster_tolerance_m").as_double();
  cluster_min_size_ = this->get_parameter("cluster_min_size").as_int();
  cluster_max_size_ = this->get_parameter("cluster_max_size").as_int();
  // TF initialization disabled
  // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { updateCameraPoseTF(); });
  // updateCameraPoseTF();

  //pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/box_detection/pointcloud", 10);

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", 10,
    std::bind(&BoxDetector::cameraInfoCallback, this, std::placeholders::_1));

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pointcloud2", rclcpp::SensorDataQoS(), std::bind(&BoxDetector::pointcloudCallback, this, std::placeholders::_1));
  filtered_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/box_detection/pointcloud_filtered", 10);
  clusters_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/box_detection/pointcloud_clusters", 10);

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
  // point cloud generation disabled; keeping images only
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
  // point cloud generation disabled; keeping images only
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



// void BoxDetector::updateCameraPoseTF()
// {
//   if (!tf_buffer_) return;
//   try {
//     auto tf = tf_buffer_->lookupTransform(world_frame_, camera_optical_frame_, tf2::TimePointZero);
//     last_camera_tf_ = tf;
//     if (!have_camera_tf_) {
//       RCLCPP_INFO(this->get_logger(), "Camera TF acquired: frame=%s height=%.3f", camera_optical_frame_.c_str(), tf.transform.translation.z);
//     }
//     have_camera_tf_ = true;
//   } catch (const tf2::TransformException& ex) {
//     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
//       "TF lookup failed (%s -> %s): %s", world_frame_.c_str(), camera_optical_frame_.c_str(), ex.what());
//     have_camera_tf_ = false;
//   }
// }

void BoxDetector::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  if (!msg) return;
  //const float min_radius = 0.20f; // radius threshold



  // Convert incoming cloud to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud_in);
  // // Save raw cloud once (dereference pointer). Remove or parameterize as needed.
  // static bool saved_once = false;
  // if (!saved_once) {
  //   if (pcl::io::savePCDFileBinary("saved_cloud.pcd", *cloud_in) == 0) {
  //     RCLCPP_INFO(this->get_logger(), "Saved initial cloud to saved_cloud.pcd (%zu points)", cloud_in->size());
  //   } else {
  //     RCLCPP_WARN(this->get_logger(), "Failed to save PCD file");
  //   }
  //   saved_once = true;
  // }

  // PassThrough on Z (height) keeping points above threshold (parameter min_height_m_)
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimitsNegative(true);  
  pass.setFilterLimits(static_cast<float>(min_height_m_), std::numeric_limits<float>::infinity());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter(*cloud_height);

  // Clustering and color assignment
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!cloud_height->empty()) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_height);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_m_);
    ec.setMinClusterSize(cluster_min_size_);
    ec.setMaxClusterSize(cluster_max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_height);
    ec.extract(cluster_indices);

    // Color palette (repeat if more clusters)
    const std::vector<std::array<uint8_t,3>> palette = {
      {255,0,0},{0,255,0},{0,0,255},{255,255,0},{255,0,255},{0,255,255},
      {255,128,0},{128,0,255},{0,128,255},{128,255,0},{255,0,128},{0,255,128}
    };
    colored->reserve(cloud_height->size());
    int cluster_id = 0;
    for (const auto &indices : cluster_indices) {
      auto color = palette[cluster_id % palette.size()];
      for (int idx : indices.indices) {
        const auto &p = (*cloud_height)[idx];
        pcl::PointXYZRGB cp; cp.x = p.x; cp.y = p.y; cp.z = p.z;
        cp.r = color[0]; cp.g = color[1]; cp.b = color[2];
        colored->push_back(cp);
      }
      ++cluster_id;
    }
    if (!colored->empty()) {
      sensor_msgs::msg::PointCloud2 clusters_msg;
      pcl::toROSMsg(*colored, clusters_msg);
      clusters_msg.header = msg->header;
      clusters_pointcloud_pub_->publish(clusters_msg);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Clusters: %zu (points=%zu)", (size_t)cluster_indices.size(), colored->size());
    }
  }




  // Convert back to ROS message
  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(*cloud_height, filtered_msg);
  filtered_msg.header = msg->header;
  filtered_pointcloud_pub_->publish(filtered_msg);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "Filtered cloud: kept %u / %zu (z>=%.2f) colored_clusters=%s", cloud_height->width * cloud_height->height,
    (size_t)(msg->width*msg->height), min_height_m_, (colored && !colored->empty())?"yes":"no");
}


} // namespace box_detection

