std_msgs/Header header
uint8 command_type_mask
uint8 RATE_SETPOINT = 1
uint8 TORQUE_SETPOINT = 2
uint8 MOTOR_SETPOINT = 3

float64 thrust_cmd
float64[] rate_cmd
float64[] torque_cmd
float64[] motor_cmd

