#!/bin/bash
set -exo pipefail 
export ROS_DOMAIN_ID={{ drone_num }}
export ROS_LOG_DIR={{ base_path }}/logs/$(date +%Y/%m/%d/%H%M%S)

mkdir -p {{ base_path }}/logs
ln -sfn $ROS_LOG_DIR {{ base_path }}/logs/latest

cd {{ ros_path }}
source /opt/ros/humble/setup.bash
source install/setup.bash

# Stagger start times: two seconds per number
<<<<<<< HEAD
sleep {{ 0 if (drone_num | int) == 0 else (2*( drone_num | int))  }}
=======
sleep {{ 2*(( drone_num | int) - 1) }}
>>>>>>> dabb432 (drone num)
# Use exec to replace the shell process with ros2 launch
# This ensures signals (SIGTERM) from systemd are properly forwarded
exec ros2 launch {{ base_path }}/launch.py 
