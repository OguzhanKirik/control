cmake_minimum_required(VERSION 3.8)
project(crtl_practice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

 # Include directories
include_directories(
  include
)


add_executable(pid_joint_node src/pid_joint_node.cpp)
ament_target_dependencies(pid_joint_node rclcpp sensor_msgs std_msgs Eigen3)
target_include_directories(pid_joint_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(kinematic_resolved_rate_node src/kinematic_resolved_rate_node.cpp)
ament_target_dependencies(kinematic_resolved_rate_node rclcpp sensor_msgs std_msgs Eigen3)
target_include_directories(kinematic_resolved_rate_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(computed_torque_node src/computed_torque_node.cpp)
ament_target_dependencies(computed_torque_node rclcpp sensor_msgs std_msgs Eigen3)
target_include_directories(computed_torque_node PRIVATE ${EIGEN3_INCLUDE_DIRS})


install(TARGETS 
      pid_joint_node
      kinematic_resolved_rate_node
      computed_torque_node   
      DESTINATION lib/${PROJECT_NAME})


ament_package()
