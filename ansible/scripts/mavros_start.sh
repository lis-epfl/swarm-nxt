set -x

export ROS_DOMAIN_ID={{ ros_domain_id }}
GCS_URL="udp://@{{ gcs_url }}:14550"

source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch gcs_url:=$GCS_URL