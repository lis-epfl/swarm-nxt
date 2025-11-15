#!/bin/bash
set -e

# Source user environment and ROS
source /home/lis/.bashrc
source /opt/ros/humble/setup.sh

# Build multi-agent planner messages first
colcon build --symlink-install --packages-select multi_agent_planner_msgs jps3d decomp_util convex_decomp_util path_finding_util voxel_grid_util decomp_ros_msgs decomp_ros_utils

# Source the built packages
source install/setup.sh

# Build main packages
colcon build --symlink-install
