#!/bin/bash
set -exo pipefail 
export ROS_DOMAIN_ID={{ drone_num }}
export ROS_LOG_DIR={{ base_path }}/logs/$(date +%Y/%m/%d/%H%M%S)

# Parse command line arguments
NOW_FLAG=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --now)
            NOW_FLAG=true
            shift
            ;;
        *)
            echo "Unknown option $1"
            exit 1
            ;;
    esac
done

mkdir -p {{ base_path }}/logs
ln -sfn $ROS_LOG_DIR {{ base_path }}/logs/latest


cd {{ ros_path }}
source /opt/ros/humble/setup.bash
source install/setup.bash

GUROBI_HOME=/opt/gurobi1003/armlinux64
PATH=$PATH:$GUROBI_HOME/bin
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GUROBI_HOME/lib

export ACADOS_SOURCE_DIR=/opt/acados
export LD_LIBRARY_PATH=/opt/acados/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/acados/interfaces/acados_template:$PYTHONPATH

# Calculate dynamic sleep time based on drone position in sorted order
DRONE_NUM={{ drone_num | int }}
# All drone numbers in the group, sorted
SORTED_DRONE_NUMS=({{ groups['drones'] | map('regex_replace', '\\.local$', '') | map('regex_search', '[0-9]+$') | map('int') | sort | join(' ') }})

# Override sleep time if --now flag is used
if [[ "$NOW_FLAG" == "true" || $DRONE_NUM -eq 0 ]]; then
    SLEEP_TIME=0
else
    # Find position of current drone in sorted list
    POSITION=-1
    for i in "${!SORTED_DRONE_NUMS[@]}"; do
        if [[ "${SORTED_DRONE_NUMS[$i]}" == "$DRONE_NUM" ]]; then
            POSITION=$i
            break
        fi
    done
    
    # Error check: drone number should be found in the sorted list
    if [[ $POSITION -eq -1 ]]; then
        echo "ERROR: Drone number $DRONE_NUM not found in sorted drone list: ${SORTED_DRONE_NUMS[*]}"
        echo "This indicates a configuration issue. Check inventory.ini and hostname."
        exit 1
    fi
    
    # Calculate sleep time: position 0 gets 2s, position 1 gets 4s, etc.
    SLEEP_TIME=$(( ($POSITION + 1) * 2 ))
fi

# Stagger start times based on sorted order
sleep $SLEEP_TIME

ros2 daemon stop

# Use exec to replace the shell process with ros2 launch
# This ensures signals (SIGTERM) from systemd are properly forwarded
exec ros2 launch {{ base_path }}/launch.py 
