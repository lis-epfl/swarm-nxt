#!/bin/bash
set -exo pipefail 
export ROS_DOMAIN_ID={{ ros_domain_id }}

cd {{ drone_ros_path }}
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch {{ drone_base_path }}/launch.yaml