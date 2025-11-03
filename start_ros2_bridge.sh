#!/bin/bash

# UR5 ROS 2 Docker Startup Script
# This script starts the ROS 2 side of the Webots-ROS bridge

echo "========================================="
echo "Starting UR5 ROS 2 Webots Bridge"
echo "========================================="

# Check if we're in the right directory
if [ ! -f "install/setup.bash" ]; then
    echo "Error: Not in ROS 2 workspace directory"
    echo "Please run this script from /home/oguz/control_ws"
    exit 1
fi

# Source the workspace
echo "Sourcing ROS 2 workspace..."
source install/setup.bash

# Kill any existing ROS processes
echo "Cleaning up existing ROS processes..."
pkill -f ros2 || true
sleep 2

# Check if the package exists
if ! ros2 pkg list | grep -q ur5_webots_simulation; then
    echo "Error: ur5_webots_simulation package not found"
    echo "Please build the workspace first with: colcon build"
    exit 1
fi

echo "Available topics after startup:"
echo "--------------------------------"

# Start the bridge node in background
echo "Starting Webots ROS 2 bridge node..."
ros2 run ur5_webots_simulation webots_ros2_bridge.py &
BRIDGE_PID=$!

# Start robot state publisher
echo "Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat install/ur5_webots_simulation/share/ur5_webots_simulation/urdf/ur5e.urdf)" &
STATE_PUB_PID=$!

# Wait a moment for nodes to start
sleep 3

echo ""
echo "========================================="
echo "ROS 2 Bridge Started Successfully!"
echo "========================================="
echo "Bridge PID: $BRIDGE_PID"
echo "State Publisher PID: $STATE_PUB_PID"
echo ""
echo "Available topics:"
ros2 topic list
echo ""
echo "To control the robot, use:"
echo "  ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \"data: [0.5, -1.0, 0.5, -1.0, 0.5, 0.0]\""
echo ""
echo "To see joint states:"
echo "  ros2 topic echo /joint_states"
echo ""
echo "Press Ctrl+C to stop all nodes"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down ROS 2 nodes..."
    kill $BRIDGE_PID 2>/dev/null || true
    kill $STATE_PUB_PID 2>/dev/null || true
    pkill -f ros2 || true
    echo "Shutdown complete."
}

# Set trap to cleanup on script exit
trap cleanup EXIT

# Wait for user to stop
wait