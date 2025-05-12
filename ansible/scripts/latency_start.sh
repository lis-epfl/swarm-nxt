#!/bin/bash
set -x

export ROS_DOMAIN_ID={{ ros_domain_id }}
source /opt/ros/humble/setup.bash
source {{ drone_ros_path }}/install/setup.bash
SAFE_HOSTNAME=$($HOSTNAME | sed s/-/_/)
ros2 run latency_checker_ros2 latency_checker --ros-args -r __ns:=$SAFE_HOSTNAME