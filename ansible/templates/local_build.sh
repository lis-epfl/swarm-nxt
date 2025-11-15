#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/humble/setup.sh

# Build main packages
colcon build --symlink-install src
