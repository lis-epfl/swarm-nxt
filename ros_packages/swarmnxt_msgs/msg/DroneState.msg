# DroneState.msg

# Standard ROS Header
std_msgs/Header header

# Drone state enumeration
uint8 IDLE=0
uint8 TAKING_OFF=1
uint8 HOVERING=2
uint8 OFFBOARD=3
uint8 LANDING=4
uint8 LAST_STATE=5

uint8 state