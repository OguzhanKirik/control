FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    # ROS2 packages
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Computer vision and PCL dependencies
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-visualization-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    # OpenCV and computer vision
    python3-opencv \
    libopencv-dev \
    libopencv-contrib-dev \
    # PCL (Point Cloud Library)
    libpcl-dev \
    pcl-tools \
    # Additional utilities
    nano \
    vim \
    htop \
    tree \
    # Development tools
    gdb \
    valgrind \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized"

# Set up workspace
WORKDIR /home/oguz/neura_tasks_ws

# Copy workspace files
COPY . /home/oguz/neura_tasks_ws/

# Update rosdep and install dependencies
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/oguz/neura_tasks_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Expose common ROS ports
EXPOSE 11311 5000 8080

# Set up shell
SHELL ["/bin/bash", "-c"]

# Default command
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /home/oguz/neura_tasks_ws/install/setup.bash && exec bash"]
