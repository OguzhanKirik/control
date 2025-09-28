from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for just the box detector node (without bag playback).
    Use this when you want to run detection on live camera feed or 
    when you're playing bags separately.
    """
    # Get package directory
    pkg_dir = get_package_share_directory('box_detection')
    
    # Box detector node
    box_detector_node = Node(
        package='box_detection',
        executable='box_detector',
        name='box_detector_node',
        parameters=[
            os.path.join(pkg_dir, 'config', 'box_detection_params.yaml')
        ],
        output='screen'
    )


    # pointcloud_node = Node(
    #         package='depth_image_proc',
    #         executable='point_cloud_xyzrgb_node',
    #         name='point_cloud_xyzrgb',
    #         remappings=[
    #             ('image_rect_color', '/camera/color/image_raw'),
    #             ('depth_registered', '/camera/aligned_depth_to_color/image_raw'),
    #             ('camera_info', '/camera/color/camera_info'),
    #             ('points', '/camera/depth/points')
    #         ],
    #         parameters=[{
    #             'qos_overrides./points.publisher.reliability': 'best_effort'
    #         }]
    # )


    ld = LaunchDescription()
    ld.add_action(box_detector_node)
    #ld.add_action(pointcloud_node)

    return ld