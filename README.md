# Smart Palletizing ROS2 Development Environment

This repository contains a ROS2 project for smart palletizing with box detection, 3D point cloud processing, and 6D pose estimation.

## Features

- 2D box detection using OpenCV
- 3D point cloud generation from RGB-D data
- RANSAC plane segmentation for face detection
- 6D pose estimation with PCA
- Real-time visualization with coordinate frame overlays
- RViz integration for 3D visualization

## Development with VS Code DevContainer

### Prerequisites

- Docker installed on your system
- VS Code with the "Dev Containers" extension

### Setup

1. Open this folder in VS Code
2. When prompted, click "Reopen in Container" or press `Ctrl+Shift+P` and select "Dev Containers: Reopen in Container"
3. VS Code will build the Docker container and set up the development environment automatically

### What's Included

The devcontainer provides:
- ROS2 Humble
- All required dependencies (OpenCV, PCL, etc.)
- VS Code extensions for ROS2, C++, Python development
- Pre-configured build and debug settings

### Building the Project

The workspace is automatically built when the container starts. To rebuild manually:

```bash
colcon build --symlink-install
```

### Running the System

#### Full System (Detection + Point Cloud + Pose Estimation)
```bash
ros2 launch box_detection box_detection_full.launch.py
```

#### Individual Components
```bash
# Box detector only
ros2 run box_detection box_detector

# Point cloud node only  
ros2 run box_detection point_cloud_node
```

### Viewing Results

#### Image Visualization
```bash
ros2 run rqt_image_view rqt_image_view /box_detection/boxes_image
ros2 run rqt_image_view rqt_image_view /point_cloud_node/box_pose_image
```

#### Topic Monitoring
```bash
# Bounding box coordinates
ros2 topic echo /box_detection/boxes_coordinates

# 6D pose data
ros2 topic echo /point_cloud_node/box_poses

# Face detection results
ros2 topic echo /point_cloud_node/box_faces
```

#### 3D Visualization with RViz
```bash
rviz2
```

Add these displays:
- PointCloud2: `/point_cloud_node/pointcloud`
- PointCloud2: `/point_cloud_node/box_pointclouds`
- MarkerArray: `/point_cloud_node/plane_markers`
- MarkerArray: `/point_cloud_node/box_coordinate_frames`

## Alternative: Docker Compose

For non-VS Code development:

```bash
# Build and run development container
docker-compose up smart-palletizing

# Run the detection system
docker-compose up box-detection
```

## Project Structure

```
├── src/box_detection/           # Main ROS2 package
│   ├── include/                 # Header files
│   ├── src/                     # Source files
│   ├── launch/                  # Launch files
│   ├── config/                  # Configuration files
│   └── scripts/                 # Utility scripts
├── .devcontainer/               # VS Code devcontainer config
├── Dockerfile                   # Container definition
├── docker-compose.yml          # Multi-container setup
└── README.md                    # This file
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/box_detection/boxes_image` | Image | 2D bounding boxes overlay |
| `/box_detection/boxes_coordinates` | String | Bounding box coordinates |
| `/point_cloud_node/pointcloud` | PointCloud2 | Full 3D point cloud |
| `/point_cloud_node/box_pointclouds` | PointCloud2 | Box-filtered point clouds |
| `/point_cloud_node/box_poses` | String | 6D pose data |
| `/point_cloud_node/box_pose_image` | Image | Coordinate systems overlay |
| `/point_cloud_node/plane_markers` | MarkerArray | Detected plane markers |
| `/point_cloud_node/box_coordinate_frames` | MarkerArray | 3D coordinate frames |

## Parameters

Key parameters in the launch files:
- `target_frame`: Robot base frame (default: "root_link")
- `camera_frame`: Camera optical frame (default: "camera_color_optical_frame")
- `noise_filter_neighbors`: Statistical filter neighbors (default: 50)
- `ransac_distance_threshold`: RANSAC plane threshold (default: 0.01m)

## Troubleshooting

### Container Build Issues
```bash
# Clean rebuild
docker-compose down
docker-compose build --no-cache
```

### ROS2 Network Issues
```bash
# Check ROS domain
echo $ROS_DOMAIN_ID

# List available topics
ros2 topic list

# Check node status
ros2 node list
```
