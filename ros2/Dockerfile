FROM ros:humble

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3.10-venv \
    python3-dev \
    ros-humble-rviz2 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    ros-humble-cv-bridge \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Source ROS installation
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Copy entrypoint script and make it executable
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-c", "tail -f /dev/null"]
