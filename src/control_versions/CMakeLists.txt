cmake_minimum_required(VERSION 3.8)
project(control_versions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


set (dependencies
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  Eigen3
  trajectory_msgs
  )

  # Include directories
include_directories(
  include
)

add_executable(ur5_cartesian_controller src/ur5_cartesian_controller.cpp)
ament_target_dependencies(ur5_cartesian_controller ${dependencies})
target_include_directories(ur5_cartesian_controller PRIVATE ${EIGEN3_INCLUDE_DIRS})


# Torque controller
add_executable(ur5_torque_controller_node src/ur5_torque_controller_node.cpp)

target_link_libraries(ur5_torque_controller_node Eigen3::Eigen)
ament_target_dependencies(ur5_torque_controller_node ${dependencies})


# MPC (torque) controller
add_executable(ur5_mpc_torque_node src/ur5_mpc_torque_node.cpp)
target_link_libraries(ur5_mpc_torque_node Eigen3::Eigen)
ament_target_dependencies(ur5_mpc_torque_node ${dependencies})

# Install
install(TARGETS
  #${PROJECT_NAME}
  ur5_cartesian_controller
  ur5_torque_controller_node
  ur5_mpc_torque_node
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# ##install
# install(TARGETS 
#   ${PROJECT_NAME}
#   box_detector
#   point_cloud_node
#   ARCHIVE DESTINATION lib 
#   LIBRARY DESTINATION lib 
#   RUNTIME DESTINATION lib/${PROJECT_NAME} 
# )



# install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
