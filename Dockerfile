FROM ros:humble-ros-base-jammy

# Install build tools and Python hardware deps available via apt
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-smbus2 \
    python3-serial \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# RPi.GPIO is not available on x86 — install a stub for building/linting
RUN pip3 install --no-cache-dir RPi.GPIO-stubs 2>/dev/null || true

# Create workspace directory
WORKDIR /ros2_ws

# Source ROS2 on every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
RUN echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc

CMD ["bash"]
