#!/bin/bash
set -e

# Source the ros installation
source /opt/ros/humble/setup.bash

# Install framegrab in editable mode for interactive development
pip install -e /framegrab

# Start the mock camera node and rviz
python3 /framegrab/ros2/sample_camera.py &
rviz2 -d /framegrab/ros2/config.rviz
exec "$@"
