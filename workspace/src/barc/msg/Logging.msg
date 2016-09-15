# This is a message to hold data for the ECU (electronic control unit)
#
# Units may vary depending on the topic
# The motor controls the speeds of the vehicle through an input torque. (For input force, divide by radius of tire) 
# The servo controls the steering angle
#
# For modeling and state estimation, motors units are [N], and servo units are [rad]
# For actuator signals, both have units of PWM angle [deg]. This relates to the duty cycle
float32 obj_val
string status
float32 solve_time
float32[] s
float32[] ey
float32[] epsi
float32[] v
float32[] a
float32[] d_f
