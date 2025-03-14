#!/bin/bash
set -x 
export ROS_DOMAIN_ID={{ ros_domain_id }}

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mocap_to_vision_pose_ros2 check_and_launch.py