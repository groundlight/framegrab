#!/bin/bash

source /opt/ros/humble/setup.bash
python3 /framegrab/ros2/sample_camera.py &
rviz2 -d /framegrab/ros2/config.rviz