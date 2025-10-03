std_msgs/Header header
uint8 command_type_mask
uint8 POSITION_SETPOINT = 1
uint8 RATE_SETPOINT = 2
uint8 PVA_SETPOINT = 3

# Mutually exclusive, position and rate setpoint shall not be used together
# Position Command
geometry_msgs/PoseStamped pose_cmd

# Rate Command (attitude setpoint)
px4_msgs/VehicleAttitudeSetpoint rate_cmd

# Position, Velocity, Acceleration Command (trajectory setpoint)
px4_msgs/TrajectorySetpoint raw_cmd