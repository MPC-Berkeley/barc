#!/usr/bin/python
import rospy
import time
from barc.msg import TimeData
from filtering import filteredSignal
from kinematic_equations import estimateAngularAcceleration, estimateVelocity, estimatePosition
from geometry_msgs.msg import Vector3
from numpy import pi, cos, sin
from input_map import angle_2_servo, servo_2_angle
import os

# input variables
d_f 	= 0
d_f_pwm	= 0
v_x_pwm = 0
test_mode = 0

# raw measurement variables
# from IMU
roll    = 0
pitch   = 0
yaw 	= 0
w_x 	= 0
w_y 	= 0
w_z 	= 0
a_x 	= 0
a_y 	= 0
a_z 	= 0

# from encoder 
v_x 	= 0
v_y 	= 0

# esc command update
def esc_callback(data):
	# update commands 
	global d_f_pwm, v_x_pwm, d_f, test_mode
	v_x_pwm 	= data.x
	d_f_pwm 	= data.y
	d_f 		= pi/180*servo_2_angle(d_f_pwm)
	test_mode 	= data.z

# imu measurement update
def imu_callback(data):
	# update measurements from IMU
	global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
	(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = data.value

# encoder measurement update
t0 	= time.time()
FL_count 		= 0
FR_count 		= 0
FL_count_prev 	= 0
FR_count_prev 	= 0
radius 			= 0.0319
dist_between_magnets 	= 2*pi*radius/4
def enc_callback(data):
	global v_x, v_y, d_f, t0 
	global FL_count, FR_count, FL_count_prev, FR_count_prev 	 
	FL_count = data.x
	FR_count = data.y

	# compute time elapsed
	tf = time.time()
	dt = tf - t0 
	
	# if enought time elapse, estimate v_x
	dt_min = 0.20
	if dt > dt_min: 
		# compute speed :  speed = distance / time 
		v_FL = (FL_count - FL_count_prev)*dist_between_magnets/dt
		v_FR = (FR_count - FR_count_prev)*dist_between_magnets/dt

		# update encoder v_x, v_y measurements
		# only valid for small slip angles, still valid for drift?
		v_x 	= (v_FL + v_FR)/2*cos(d_f)
		v_y 	= (v_FL + v_FR)/2*sin(d_f)

		# update old data
		FL_count_prev = FL_count
		FR_count_prev = FR_count
		t0 	 = time.time()

# state estimation node 
def state_estimation():
	# initialize node
	rospy.init_node('state_estimation', anonymous=True)
	rospy.Subscriber('imu_data', TimeData, imu_callback)
	rospy.Subscriber('enc_data', Vector3, enc_callback)
	rospy.Subscriber('esc_cmd', Vector3, esc_callback)
	state_pub 	= rospy.Publisher('state_estimate', Vector3, queue_size = 10)

	# get system parameters
	test_sel 	= rospy.get_param("auto_node/test_sel")
	test_opt  	= {0 : "CRC_",
				   1 : "STR_",
				   2 : "SWP_",
				   3 : "DLC_",
				   4 : "LQR_"}
	test_name 	= test_opt.get(test_sel)

	# set node rate
	loop_rate 	= 50
	dt 			= 1.0 / loop_rate
	rate 		= rospy.Rate(loop_rate)

	#  Vehicle parameters
	a       = 0.14          # distance from CoG to front axel [m]
	b       = 0.14          # distance from CoG to rear axel [m]
	vh_dims = (a,b)

	# filter parameters
	# aph   := smoothing factor, (all filtered)   0 <=   aph   <= 1  (no filter)
	# n     := size of moving average block
	p_ax        = 0.3        
	p_ay        = p_ax      
	p_yaw       = 1 	   
	p_wz        = 0.8     
	n           = 25     

	# measurement signal
	# velocity data in the  frame
	# position/velocuty data in the global frame
	imu_data   	= filteredSignal(y0 = [0,0,0,0], a = [p_ax, p_ay, p_yaw, p_wz], method = 'lp')
	v_BF     	= filteredSignal(y0 = [0,0], n = n, method = 'mvg')
	X_GF     	= filteredSignal(y0 = [0,0,0,0], method = None)

	# variable for angular acceleration estimate
	w_z_prev 	= 0

	# save data to file
	date 				= time.strftime("%Y.%m.%d")
	BASE_PATH   		= "/home/odroid/Data/" + date + "/"

	# create directory if it doesn't exist
	if not os.path.exists(BASE_PATH):
		os.makedirs(BASE_PATH)

	# create file
	data_file_name   	= BASE_PATH + test_name + time.strftime("%H.%M.%S") + '.csv'
	data_file     		= open(data_file_name, 'a')
	data_file.write('t_s,test_mode,roll_imu,pitch_imu,yaw_imu,w_x_imu,w_y_imu,w_z_imu,a_x_imu,a_y_imu,a_z_imu,FL_enc_count,FR_enc_count,v_x_enc,v_y_enc,v_x_pwm,d_f_pwm,d_f,v_x_hat,v_y_hat,w_z_hat\n')
	t0 				= time.time()

	while not rospy.is_shutdown():
		# measurements from imu and encoder
		global test_mode, roll, pitch, yaw, w_x, w_y, w_z, a_x, a_y, a_z
		global FL_count, FR_count, v_x, v_y, v_x_pwm, d_f_pwm, d_f

		# filter signals of IMU
		imu_data.update([a_x, a_y, yaw, w_z])

		# estimate angular acceleration  dwz
		(dwz, w_z_prev)  = estimateAngularAcceleration(imu_data, w_z_prev, dt)

		# estimate velocity in body frame
		# estimate position in global frame
		estimateVelocity(imu_data, v_BF, v_x, dwz, vh_dims, dt)
		estimatePosition(v_BF, X_GF, yaw, v_x, dt)

		# get state estimate
		(v_x_hat, v_y_hat) 	= v_BF.getFilteredSignal()
		(_,_,_, w_z_hat) 	= imu_data.getFilteredSignal() 

		# publish state estimate
		state_pub.publish( Vector3(v_x_hat, v_y_hat, w_z_hat) )

		# save data (maybe should use rosbag in the future)
		t  	= time.time() - t0
		all_data = (t,test_mode,roll,pitch,yaw,w_x,w_y,w_z,a_x,a_y,a_z,FL_count,FR_count,v_x,v_y,v_x_pwm,d_f_pwm,d_f,v_x_hat,v_y_hat,w_z_hat)
		N = len(all_data)
		str_fmt = '%.4f,'*N
		data_file.write( (str_fmt[0:-1]+'\n') % all_data)

		# wait
		rate.sleep()

if __name__ == '__main__':
	try:
		state_estimation()
	except rospy.ROSInterruptException:
		pass
