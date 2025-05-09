#!/bin/bash
set -e

# Source the ros installation
source /opt/ros/humble/setup.bash

# Install framegrab in editable mode for interactive development
pip install -e /framegrab

exec "$@"
