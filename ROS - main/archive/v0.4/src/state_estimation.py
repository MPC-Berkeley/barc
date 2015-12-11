#!/usr/bin/python
import rospy
import time
from barc.msg import TimeData
from filtering import filteredSignal
from kinematic_equations import estimateAngularAcceleration, estimateVelocity, estimatePosition
from geometry_msgs.msg import Vector3

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

# encoder and steering angle readings
v_x_ENC 	= 0
d_f 		= 0
w_z_prev 	= 0

def imu_callback(data):
	# load global variable/objects
	global imu_data, v_BF, X_GF
	global w_z_prev, d_f, v_x_ENC

	# get data
	dt 			= 1.0/20   		# imu_data_acq node is publishing at 50 Hz
	raw_data	= data.value
	(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = raw_data

	# filter signals of IMU
	imu_data.update([a_x, a_y, yaw, w_z])

	# estimate angular acceleration  dwz
	(dwz, w_z_prev)  = estimateAngularAcceleration(imu_data, w_z_prev, dt)

	# estimate velocity in body frame
	# estimate position in global frame
	estimateVelocity(imu_data, v_BF, v_x_ENC, dwz, d_f, vh_dims, dt)
	estimatePosition(v_BF, X_GF, yaw, v_x_ENC, dt)

def enc_callback(data):
	global v_x_ENC, d_f
	v_x_ENC = data.x
	d_f 	= data.z

def state_estimation():
	# initialize node
	rospy.init_node('state_estimation', anonymous=True)
	rospy.Subscriber('imu_data', TimeData, imu_callback)
	rospy.Subscriber('enc_data', Vector3, enc_callback)
	state_pub 	= rospy.Publisher('state_estimate', Vector3, queue_size = 10)

	# set node rate
	loop_rate 	= 20
	rate 		= rospy.Rate(loop_rate)

	# load global objects
	global v_BF, imu_data

	while not rospy.is_shutdown():
		# get state estimate
		(v_x, v_y) 	= v_BF.getFilteredSignal()
		(_,_,_, w_z) 	= imu_data.getFilteredSignal() 

		# publish state estimate
		state_pub.publish( Vector3(v_x, v_y, w_z) )
		rate.sleep()
	

if __name__ == '__main__':
	try:
		state_estimation()
	except rospy.ROSInterruptException:
		pass
