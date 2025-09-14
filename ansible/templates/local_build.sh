#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/humble/setup.sh

# Build main packages
colcon build --base-paths src

# Build multi-agent planner messages
colcon build --base-paths multi_agent_pkgs --packages-select multi_agent_planner_msgs