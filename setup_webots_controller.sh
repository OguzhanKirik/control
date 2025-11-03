#!/bin/bash

# UR5 Webots Controller Startup Script (Mac)
# This script helps set up and run the Webots side of the simulation

echo "========================================="
echo "UR5 Webots Controller Setup (Mac)"
echo "========================================="

WEBOTS_CONTROLLERS_DIR="$HOME/Documents/Webots/controllers"
CONTROLLER_NAME="ur5_ros2_controller"
CONTROLLER_SRC="/home/oguz/control_ws/src/ur5_webots_simulation/controllers/$CONTROLLER_NAME"

echo "Setting up Webots controller..."

# Create controllers directory if it doesn't exist
if [ ! -d "$WEBOTS_CONTROLLERS_DIR" ]; then
    echo "Creating Webots controllers directory: $WEBOTS_CONTROLLERS_DIR"
    mkdir -p "$WEBOTS_CONTROLLERS_DIR"
fi

# Copy controller to Webots directory
if [ -d "$CONTROLLER_SRC" ]; then
    echo "Copying controller from $CONTROLLER_SRC to $WEBOTS_CONTROLLERS_DIR/"
    cp -r "$CONTROLLER_SRC" "$WEBOTS_CONTROLLERS_DIR/"
    echo "Controller copied successfully!"
else
    echo "Error: Controller source directory not found at $CONTROLLER_SRC"
    echo "Please make sure you're running this from the Docker container with the workspace mounted"
    exit 1
fi

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is not installed or not in PATH"
    echo "Please install Python 3 to run the external controller"
    exit 1
fi

echo ""
echo "========================================="
echo "Setup Complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Open Webots on your Mac"
echo "2. Load the world file: ur5e_simple_external.wbt"
echo "3. The robot should automatically connect to the external controller"
echo "4. Check the Webots console for connection messages"
echo ""
echo "Controller location: $WEBOTS_CONTROLLERS_DIR/$CONTROLLER_NAME/"
echo ""
echo "If you want to run the controller manually (for debugging):"
echo "  cd $WEBOTS_CONTROLLERS_DIR/$CONTROLLER_NAME/"
echo "  python3 ur5_ros2_controller.py"
echo ""

# Optionally run the controller in standalone mode for testing
read -p "Do you want to test the controller in standalone mode? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Running controller in standalone mode..."
    echo "This will test the TCP server without Webots"
    echo "Press Ctrl+C to stop"
    echo ""
    cd "$WEBOTS_CONTROLLERS_DIR/$CONTROLLER_NAME/"
    python3 ur5_ros2_controller.py
fi