#!/bin/bash
set -exo pipefail 
export ROS_DOMAIN_ID={{ ros_domain_id }}

cd {{ ros_path }}
source /opt/ros/humble/setup.bash
source install/setup.bash


ros2 launch {{ base_path }}/launch.py &
ros2 run mocap_to_vision_ros2 mocap_to_vision_pose_ros2 check_and_launch.py --namespace {{ ns }} --mocap-topic  "/optitrack_multiplexer_node/rigid_body/{{ ansible_hostname }}"