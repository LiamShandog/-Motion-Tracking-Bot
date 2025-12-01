# Multi-stage Dockerfile for Raspberry Pi 5 (ARM64)
FROM ros:humble-ros-base

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies (pigpio daemon will be provided on the host Pi)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-pip \
      alsa-utils \
      git \
      vim && \
    rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace

# Copy the entire package (not just the module)
COPY . /workspace/src/motion_tracking_bot

# Install rosdep and resolve dependencies
RUN rosdep init && rosdep update && \
    rosdep install --from-paths /workspace/src --ignore-src -r -y

# Install Python dependencies from requirements.txt
RUN pip3 install --no-cache-dir -r /workspace/src/motion_tracking_bot/requirements.txt

# Build the ROS package
RUN . /opt/ros/humble/setup.sh && \
    cd /workspace && \
    colcon build --symlink-install

# Setup bashrc for interactive shells
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /workspace/install/setup.bash' >> /root/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /workspace/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
