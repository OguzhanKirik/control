#include "box_detection/point_cloud_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<box_detection::PointCloudNode>();
  node->initializeImageTransport();

  RCLCPP_INFO(node->get_logger(), "Point Cloud Node started");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
