from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for both box detector and point cloud nodes.
    The box detector finds 2D bounding boxes, and the point cloud node 
    generates full point clouds plus extracts 3D points for each detected box.
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

    # Point cloud node with box extraction and plane segmentation
    pointcloud_node = Node(
            package='box_detection',
            executable='point_cloud_node',
            name='point_cloud_node',
            output='screen',
            parameters=[{
                'target_frame': 'root_link',
                'camera_frame': 'camera_color_optical_frame',
                'noise_filter_neighbors': 50,
                'noise_filter_std_dev': 1.0,
                'ransac_distance_threshold': 0.01,  # 1cm threshold for RANSAC
                'ransac_max_iterations': 1000,
                'plane_area_threshold': 0.001  # Minimum plane area in m²
    }]
    )

    ld = LaunchDescription()
    ld.add_action(box_detector_node)
    ld.add_action(pointcloud_node)

    return ld

