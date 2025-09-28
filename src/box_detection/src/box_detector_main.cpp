#include<memory>
#include "box_detection/box_detector_node.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<box_detection::BoxDetector>();
    node->initializeImageTransport();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}