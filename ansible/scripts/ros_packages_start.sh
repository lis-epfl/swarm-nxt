#!/bin/bash
set -x 
export ROS_DOMAIN_ID={{ ros_domain_id }}

cd {{ drone_ros_path }}
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mocap_to_vision_pose_ros2 check_and_launch.py &

SAFE_HOSTNAME=$($HOSTNAME | sed s/-/_/)
ros2 run latency_checker_ros2 latency_checker --ros-args -r __ns:=$SAFE_HOSTNAME & 