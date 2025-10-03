std_msgs/Header header
uint8 command_type_mask
uint8 POSITION_SETPOINT = 1
uint8 RATE_SETPOINT = 2
uint8 PVA_SETPOINT = 3

# Trajectory setpoint command (used for all command types)
px4_msgs/TrajectorySetpoint cmd