#!/bin/bash
set -e

# Source user environment and ROS
source /home/lis/.bashrc
source /opt/ros/humble/setup.sh

# Build multi-agent planner messages first
colcon build --symlink-install --base-paths multi_agent_pkgs --packages-select multi_agent_planner_msgs

# Source the built packages
source install/setup.sh

# Build main packages
colcon build --symlink-install --base-paths src

# Build additional multi-agent packages (decomposition and utilities)
colcon build --symlink-install --base-paths multi_agent_pkgs --packages-select jps3d decomp_util convex_decomp_util path_finding_util voxel_grid_util decomp_ros_msgs decomp_ros_utils

# Source again after utilities build
source install/setup.sh

# Build final multi-agent packages (environment and planner)
colcon build --symlink-install --base-paths multi_agent_pkgs --packages-select env_builder_msgs env_builder mapping_util multi_agent_planner