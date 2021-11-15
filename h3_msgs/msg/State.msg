# name: The robot name defined in hardware configuration file.
# joint_control_type: 
# 0: No control selected - Motor disabled.
# 1: Position controller Selected.
# 2: Stiffness controller selected.
# 3: Torque controller selected.
# 4: Motor disabled.
# 5: Motor stopped.
# 6: Open loop enable.
# 7: Task controller enable, control_type for all joints should be set to 7.
# status: Useful when control_type selected is task controller enable.
# 0: Standing up.
# 1-10: Walking speed from 1 to 10.
# 11: Joints in passive mode (motors disabled).
# 12: Joints in compliant mode (zero torque mode).
# 13: Stopping walk.
# 14: Performing stand up.
# 15: Performing sit down.
# 16: Seated down.
# 17: Performing left step.
# 18: Performing right step.
# 19: All joints blocked.
# 21: Right hip failure.
# 22: Right knee failure.
# 23: Right hip failure.
# 24: Right hip failure.
# 25: Right hip failure.
# 26: Right hip failure.
# Command_app:
# 0: The Main Controller wiil process the command received from ROS hardware interface.
# 1: The Main Controller will process the command received from Bluetooth interface.
# runtime: Main Controller internal clock.
# connection_status:
# 0: Main controller not connected.
# 1: Main controller connected.
# recording_status:
# 0: No data is being recorded.
# 1: Recording data.
# 2: Waiting for an input trigger to start data recording.
# 3: Waiting for an output trigger to start data recording.
std_msgs/Header header
string name 
uint8 status
string[] joint_name
uint8[] joint_control_type
float64[] joint_position
float64[] joint_velocity
float64[] joint_motor_torque
float64[] joint_torque_sensor
float64 battery_voltage
uint8 right_heel_sensor
uint8 right_toe_sensor
uint8 left_heel_sensor
uint8 left_toe_sensor
uint8 trigger_input
uint8 command_app
uint8 trigger_counter
uint8 runtime_min
uint8 runtime_sec
uint8 runtime_csec
uint8 connection_status
uint8 recording_status


