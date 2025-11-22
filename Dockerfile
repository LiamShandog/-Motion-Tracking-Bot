# Multi-stage Dockerfile for Raspberry Pi 5 (ARM64)
# Stage 1: Builder
FROM ros:humble-ros-base AS builder

# Set working directory
WORKDIR /workspace

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy the package source
COPY . /workspace/src/motion_tracking_bot

# Initialize rosdep and install system dependencies
RUN rosdep init && rosdep update && \
    rosdep install --from-paths /workspace/src --ignore-src -r -y

# Build the ROS package
RUN . /opt/ros/humble/setup.sh && \
    cd /workspace && \
    colcon build --symlink-install

# Stage 2: Runtime
FROM ros:humble-ros-base

# Set working directory
WORKDIR /workspace

# Install runtime dependencies only
RUN apt-get update && apt-get install -y \
    pigpio \
    python3-pigpio \
    alsa-utils \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy built package from builder stage
COPY --from=builder /workspace/install /workspace/install
COPY --from=builder /workspace/src /workspace/src

# Install Python pip dependencies
RUN pip install --no-cache-dir pigpio>=1.78

# Setup ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

# Create entrypoint script
RUN mkdir -p /workspace/scripts && \
    echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /workspace/install/setup.bash\n\
\n\
# Wait for pigpiod to be ready if running in container\n\
if [ -z "$SKIP_PIGPIOD_CHECK" ]; then\n\
  echo "Waiting for pigpiod to be ready..."\n\
  for i in {1..30}; do\n\
    if pigs t >/dev/null 2>&1; then\n\
      echo "pigpiod is ready!"\n\
      break\n\
    fi\n\
    echo "Attempt $i/30: pigpiod not ready, waiting..."\n\
    sleep 1\n\
  done\n\
fi\n\
\n\
exec "$@"' > /workspace/scripts/entrypoint.sh && \
    chmod +x /workspace/scripts/entrypoint.sh

ENTRYPOINT ["/workspace/scripts/entrypoint.sh"]

# Default: launch the motion tracking nodes
CMD ["ros2", "launch", "motion_tracking_bot", "motion_tracking_bot.launch.py"]
