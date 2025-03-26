#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

pip install -e /framegrab

# Run both in background
python3 /framegrab/ros2/sample_camera.py &
rviz2 -d /framegrab/ros2/config.rviz
exec "$@"
