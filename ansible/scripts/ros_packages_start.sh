#!/bin/bash
set -exo pipefail 
export ROS_DOMAIN_ID={{ ros_domain_id }}
export ROS_LOG_DIR={{ base_path }}/logs/$(date +%Y/%m/%d/%H%M%S)

mkdir -p {{ base_path }}/logs
ln -sfn $ROS_LOG_DIR {{ base_path }}/logs/latest

cd {{ ros_path }}
source /opt/ros/humble/setup.bash
source install/setup.bash


ros2 launch {{ base_path }}/launch.py 