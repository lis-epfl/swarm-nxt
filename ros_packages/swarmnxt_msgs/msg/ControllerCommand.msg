std_msgs/Header header
uint8 command_type_mask
uint8 POSITION_SETPOINT = 1
uint8 RATE_SETPOINT = 2

# Mutually exclusive, position and rate setpoint shall not be used together
# Position Command
geometry_msgs/PoseStamped pose_cmd

# Rate Command
mavros_msgs/AttitudeTarget rate_cmd