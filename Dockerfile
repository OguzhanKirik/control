# Use the official ROS 2 Humble base image
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE_DIR=/home/oguz/control_ws

# Install essential development tools and MoveIt
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    vim \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-core \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-common \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-xacro \
    ros-humble-py-binding-tools \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create workspace directory
RUN mkdir -p $WORKSPACE_DIR/src

# Set up ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Set working directory
WORKDIR $WORKSPACE_DIR

# Set default command
CMD ["/bin/bash"]