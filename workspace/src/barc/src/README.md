# Nodes
*auto_mode*    		   : sends commands to the servo, motor <br />
*imu_data_acq*         : collects data from the myAHRS+ IMU sensor <br />
*state_estimation*     : estimates the state given IMU data, encoder reading, and steering angle <br />

# Topics 
*imu_data*             : raw data (roll, pitch, yaw, a_x, a_y, a_x, w_x, w_y, w_z) from IMU  <br />
*enc_data*             : steering angle from servo and v_x from encoder  (d_F, v_x) <br />
*state_estimation*     : estimate of state Z = [ v_x, v_y, w_z ] <br />
*esc_cmd* 			   : motor and servo commands

# Modules
*imu_interface*       : establish communication with the myAHRS+ IMU sensor <br />
*filtering*           : filter signals from sensors / state estimation  <br />
*kinematic_equations* : compute velocity and position from kinematic equations and coordinate transformation <br />
*maneuvers* 		  : various maneuvers for driving tests (e.g. Circular test, Straight test, Double Lange Change, etc.)