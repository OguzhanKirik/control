# Smart Palletizing ROS2 Development Environment

This repository contains a ROS2 project for smart palletizing with box detection, 3D point cloud processing, and 6D pose estimation.

## Algorithm Components

### 1. 2D Boxes Detection

The **box_detector** node is dedicated to detecting boxes in RGB camera images using OpenCV-based computer vision techniques. This component serves as the foundation for the entire pipeline by identifying regions of interest that potentially contain boxes.

**Key Features:**
- Uses traditional computer vision algorithms (edge detection, contour analysis, morphological operations)
- Outputs 2D bounding box coordinates as `(x, y, width, height)` format
- Publishes results to `/box_detection/boxes_coordinates` topic as string format
- Provides visual feedback through annotated images on `/box_detection/boxes_image` topic
- Serves as input for 3D point cloud extraction and pose estimation

**Detection Process Steps:**

1. **Input Validation** - Check availability of color image, depth image, and camera info
2. **Depth Image Preprocessing** - Normalize 16-bit depth data to 8-bit for edge detection
3. **Depth Edge Detection** - Apply Canny edge detection on depth discontinuities
4. **Color Edge Detection** - Convert to grayscale and apply Canny edge detection
5. **Edge Fusion** - Combine depth and color edges using bitwise OR operation
6. **Morphological Processing** - Apply closing and dilation to connect fragmented edges
7. **Contour Detection** - Find all contours in the processed edge image
8. **Contour Filtering** - Filter by area, size, and aspect ratio to remove false positives
9. **Bounding Box Generation** - Create rectangular bounding boxes around valid contours
10. **Visualization & Publishing** - Annotate image with colored boxes and publish coordinates

### 2. Planar Patches Detection (3D)

The **segmentPlanesInBoxes** function in the point_cloud_node performs direct face detection using dimensional analysis instead of traditional RANSAC plane segmentation. This approach is optimized for box detection with known dimensions.

**Key Features:**
- **Direct Dimensional Analysis:** Uses `getMinMax3D()` to extract bounding box dimensions from 3D point clouds
- **Known Box Matching:** Compares measured dimensions against predefined box specifications (Small_Box: 9.5×25×34cm, Medium_Box: 10×15.5×25.5cm)
- **Face Type Classification:** Determines which face of the box is visible (e.g., Small_Box_(a,b), Medium_Box_(b,c))
- **Occlusion Handling:** Accounts for partially occluded boxes by analyzing visible vs. hidden dimensions
- **Confidence Scoring:** Provides confidence levels based on dimensional matching accuracy

**Face Detection Process Steps:**

1. **Point Cloud Validation** - Check if box point cloud contains sufficient points for analysis
2. **Bounding Box Extraction** - Use `getMinMax3D()` to find 3D bounding box limits (min_pt, max_pt)
3. **Dimension Calculation** - Calculate three dimensions: X, Y, Z extents from bounding box
4. **Dimension Sorting** - Sort dimensions to identify largest (visible_dim1), second largest (visible_dim2), and smallest (hidden_dim)
5. **Known Box Matching** - Compare measured dimensions against predefined box specifications
6. **Face Type Determination** - Identify which face is visible based on dimension combinations (a,b), (a,c), or (b,c)
7. **Occlusion Analysis** - Account for partially hidden dimensions using tolerance thresholds
8. **Confidence Calculation** - Generate confidence score based on dimensional matching accuracy
9. **Face Classification** - Output face type (e.g., "Small_Box_(b,c)") with confidence level
10. **Visualization Publishing** - Generate markers and publish face detection results

### 3. Point Cloud Post Processing

The **applyRANSACNoiseRemoval** function implements RANSAC-based filtering specifically for noise removal and point cloud quality improvement, not for face detection.

**RANSAC Approach:**
- **Purpose:** Remove noise and outliers from point clouds to improve pose estimation accuracy
- **Implementation:** Uses PCL's RANSAC plane segmentation (`pcl::SACSegmentation`)
- **Parameters:** 
  - Distance threshold: 0.01m (configurable via `ransac_distance_threshold`)
  - Max iterations: 1000 (configurable via `ransac_max_iterations`)
- **Process:** 
  1. Applies RANSAC to find the dominant plane in the point cloud
  2. Extracts plane inliers as the "filtered" point cloud
  3. Returns cleaned points that belong to the main plane surface
- **Fallback:** If plane detection fails or finds insufficient points, returns original cloud
- **Quality Control:** Ensures minimum point density for reliable pose estimation

### 4. Box Pose Estimation

The **estimate6DPoseAndVisualize** function implements a sophisticated 6D pose estimation pipeline combining face detection constraints with PCA-based orientation estimation.

**6D Pose Estimation Steps:**

1. **Point Cloud Preprocessing:**
   - Apply RANSAC noise removal using `applyRANSACNoiseRemoval()`
   - Validate sufficient point density (minimum 30 points)

2. **Face Detection Integration:**
   - Call `determineFaceType()` to identify visible box face
   - Extract face confidence and dimensional constraints
   - Use known box specifications for orientation constraints

3. **Orientation Estimation:**
   - **Face-Constrained PCA:** If face detection confidence > 50%, use `estimateOrientationWithFaceConstraints()`
   - **Fallback PCA:** If face detection fails, use basic `estimateOrientationWithPCA()`
   - Apply geometric constraints based on detected face type

4. **Translation Calculation:**
   - Compute point cloud centroid using `pcl::compute3DCentroid()`
   - Refine centroid using `refineCentroidWithGeometry()` with known box dimensions
   - Account for box geometric center vs. point cloud centroid offset

5. **Pose Refinement:**
   - Use known box dimensions from face detection results
   - Apply dimensional constraints to improve accuracy
   - Convert rotation matrix to quaternion representation

6. **Output Generation:**
   - **Position:** 3D translation (x, y, z) in target coordinate frame
   - **Orientation:** Quaternion (x, y, z, w) representing 3D rotation
   - **Metadata:** Box type, confidence, estimated dimensions
   - **Visualization:** Generate coordinate frame markers for RViz


## Features

- 2D box detection using OpenCV
- 3D point cloud generation from RGB-D data
- RANSAC-based point cloud filtering for noise removal
- Direct face detection using bounding box dimensions
- 6D pose estimation with face-constrained PCA
- Real-time visualization with coordinate frame overlays
- RViz integration for 3D visualization
- Support for multiple box types (Small_Box: 9.5×25×34cm, Medium_Box: 10×15.5×25.5cm)

## Development with VS Code DevContainer

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

#### Play Rosbag ROS 2

```bash
ros2 bag play smart_palletizing_data_ros2
```

#### Individual Components
```bash
# Box detector only
ros2 run box_detection box_detector

# Point cloud node only  
ros2 run box_detection point_cloud_node
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

## Converting ROS1 .bag to ROS2

This guide shows how to convert a ROS1 `.bag` file into a ROS2-compatible rosbag2 (sqlite3 `.db3`).

### Steps

1. **Install rosbags tool**
```bash
python3 -m pip install --user "rosbags>=0.9.11"
```

2. **Run the converter**
```bash
python3 -m rosbags.convert --src smart_palletizing_data.bag --dst smart_palletizing_data_ros2
```

3. **Play in ROS 2**
```bash
ros2 bag info smart_palletizing_data_ros2
ros2 bag play smart_palletizing_data_ros2
```

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
│   │   └── box_detection/       # Package-specific headers
│   ├── src/                     # Source files
│   │   ├── box_detection/       # Main implementation
│   │   │   └── point_cloud_node.cpp  # Point cloud processing and pose estimation
│   │   └── box_detector.cpp     # 2D box detection
│   ├── launch/                  # Launch files
│   │   └── box_detection_full.launch.py  # Complete system launch
│   ├── config/                  # Configuration files
│   └── scripts/                 # Utility scripts
├── .devcontainer/               # VS Code devcontainer config
├── Dockerfile                   # Container definition
├── docker-compose.yml          # Multi-container setup
└── README.md                    # This file
```

## Key Functions

### Point Cloud Processing
- `applyRANSACNoiseRemoval()`: RANSAC-based point cloud filtering
- `segmentPlanesInBoxes()`: Direct face detection using dimensional analysis
- `estimate6DPoseAndVisualize()`: 6D pose estimation with face constraints
- `determineFaceType()`: Analyze visible box faces based on dimensions

### Face Detection
- Direct dimensional analysis using `getMinMax3D`
- No RANSAC plane segmentation required for face detection
- Face constraints applied to PCA orientation estimation

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

## Algorithm Overview

