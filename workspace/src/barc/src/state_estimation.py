#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu)  Development of the web-server app Dator was 
# based on an open source project by Bruce Wootton, with contributions from 
# Kiet Lam (kiet.lam@berkeley.edu)   
# ---------------------------------------------------------------------------

import rospy
import time
import os
import json
from numpy import pi, cos, sin
from geometry_msgs.msg import Vector3
from input_map import angle_2_servo, servo_2_angle
from observers import kinematicLuembergerObserver
from data_service.srv import *
from data_service.msg import *

import numpy as np

# input variables
d_f 	= 0
d_f_pwm	= 0
v_x_pwm = 0

# raw measurement variables
# from IMU
roll    = 0
pitch   = 0
yaw 	= 0
w_x 	= 0
w_y 	= 0
w_z 	= 0
a_x_imu 	= 0
a_y_imu 	= 0
a_z_imu 	= 0

# from encoder 
v_x_enc 	= 0
t0 	        = time.time()
n_FL	    = 0                 # counts in the front left tire
n_FR 	    = 0                 # counts in the front right tire
n_FL_prev 	= 0                 
n_FR_prev 	= 0
r_tire 		= 0.0319            # radius from tire center to perimeter along magnets
dx_magnets 	= 2*pi*r_tire/4     # distance between magnets

# esc command update
def esc_callback(data):
	global d_f_pwm, v_x_pwm, d_f
	v_x_pwm 	= data.x
	d_f_pwm 	= data.y
	d_f 		= pi/180*servo_2_angle(d_f_pwm)

# imu measurement update
def imu_callback(data):
	global roll, pitch, yaw, a_x_imu, a_y_imu, a_z_imu, w_x, w_y, w_z
	(roll, pitch, yaw, a_x_imu, a_y_imu, a_z_imu, w_x, w_y, w_z) = data.value

# encoder measurement update
def enc_callback(data):
	global v_x_enc, d_f, t0 
	global n_FL, n_FR, n_FL_prev, n_FR_prev 	 

	n_FL = data.x
	n_FR = data.y

	# compute time elapsed
	tf = time.time()
	dt = tf - t0 
	
	# if enough time elapse has elapsed, estimate v_x
	dt_min = 0.20
	if dt > dt_min: 
		# compute speed :  speed = distance / time 
		v_FL = (n_FL- n_FL_prev)*dx_magnets
		v_FR = (n_FR- n_FR_prev)*dx_magnets

		# update encoder v_x, v_y measurements
		# only valid for small slip angles, still valid for drift?
		v_x_enc 	= (v_FL + v_FR)/2*cos(d_f)

		# update old data
		n_FL_prev = n_FL
		n_FR_prev = n_FR
		t0 	 = time.time()


# state estimation node 
def state_estimation():
	# initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu_data', TimeData, imu_callback)
    rospy.Subscriber('enc_data', Vector3, enc_callback)
    rospy.Subscriber('esc_cmd', Vector3, esc_callback)
    state_pub 	= rospy.Publisher('state_estimate', Vector3, queue_size = 10)

	# get system parameters
    username = rospy.get_param("auto_node/user")
    experiment_sel 	= rospy.get_param("auto_node/experiment_sel")
    experiment_opt 	= {0 : "Circular",
				       1 : "Straight",
				       2 : "SineSweep",
				       3 : "DoubleLaneChange",
				       4 : "LQR"}
    experiment_type = experiment_opt.get(experiment_sel)
    signal_ID = username + "-" + experiment_type
    experiment_name = 'rc_car_ex1'

	# get vehicle dimension parameters
    # note, the imu is installed at the front axel
    L_a = rospy.get_param("state_estimation/L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("state_estimation/L_b")       # distance from CoG to rear axel

	# set node rate
    loop_rate 	= 50
    dt 		    = 1.0 / loop_rate
    rate 		= rospy.Rate(loop_rate)

	# filter parameters
    aph = 3             # parameter to tune estimation error dynamics

    ## Open file to save data
    date 				= time.strftime("%Y.%m.%d")
    BASE_PATH   		= "/home/odroid/Data/" + date + "/"
	# create directory if it doesn't exist
    if not os.path.exists(BASE_PATH):
        os.makedirs(BASE_PATH)
    data_file_name   	= BASE_PATH + signal_ID + '-' + time.strftime("%H.%M.%S") + '.csv'
    data_file     		= open(data_file_name, 'a')
    data_file.write('t,roll,pitch,yaw,w_x,w_y,w_z,a_x_imu,a_y_imu,a_z_imu,n_FL,n_FR,v_x_pwm,d_f_pwm,d_f,vhat_x,vhat_y,what_z\n')
    t0 				= time.time()

    # serialization variables
    samples_buffer_length = 50
    samples_counter = 0
    data_to_flush = []
    timestamps = []
    send_data = rospy.ServiceProxy('send_data', DataForward)
    
    # estimation variables
    vhat_x = 0      # longitudinal velocity estimate
    vhat_y = 0      # lateral velocity estimate
    what_z = 0      # yaw rate estimate

    while not rospy.is_shutdown():
        samples_counter += 1

		# signals from inertial measurement unit, encoder, and control module
        global roll, pitch, yaw, w_x, w_y, w_z, a_x_imu, a_y_imu, a_z_imu
        global n_FL, n_FR, v_x_enc
        global v_x_pwm, d_f_pwm, d_f

		# publish state estimate
        state_pub.publish( Vector3(vhat_x, vhat_y, w_z) )

		# save data (maybe should use rosbag in the future)
        t  	= time.time() - t0
        all_data = [t,roll,pitch,yaw,w_x,w_y,w_z,a_x_imu,a_y_imu,a_z_imu,n_FL,n_FR,v_x_pwm,d_f_pwm,d_f,vhat_x,vhat_y,what_z]
        timestamps.append(t)
        data_to_flush.append(all_data)


	# 	# save to CSV
        # N = len(all_data)
        # str_fmt = '%.4f,'*N
        # data_file.write( (str_fmt[0:-1]+'\n') % tuple(all_data))

        print samples_counter

        # do the service command asynchronously, right now this is a blocking call
        if samples_counter == samples_buffer_length:
            data = np.array(data_to_flush)
            send_all_data(data, timestamps, send_data, experiment_name)

            timestamps = []
            data_to_flush = []
            samples_counter = 0

        # assuming imu is at the center of the front axel
        # perform coordinate transformation from imu frame to vehicle body frame (at CoG)
        a_x = a_x_imu + L_a*w_z**2
        a_y = a_y_imu
        
        # update state estimate
        (vhat_x, vhat_y) = kinematicLuembergerObserver(vhat_x, vhat_y, w_z, a_x, a_y, v_x_enc, aph, dt)
        
		# wait
        rate.sleep()


# Reduce duplication somehow?
def send_all_data(data, timestamps, send_data, experiment_name):

    print data
    time_signal = TimeSignal()
    time_signal.timestamps = timestamps

    # idx = np.array([1, 1])
    # time_signal.name = 't'
    # time_signal.signal = json.dumps([data[:, 1].flatten().tolist(), data[:, 1].flatten().tolist()])
    # send_data(time_signal, None, experiment_name)

    time_signal.name = 'roll'
    idx = np.array([1])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'pitch'
    idx = np.array([2])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'yaw'
    idx = np.array([3])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'w_x'
    idx = np.array([4])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'w_y'
    idx = np.array([5])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'w_z'
    idx = np.array([6])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'a_x_imu'
    idx = np.array([7])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'a_y_imu'
    idx = np.array([8])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'a_z_imu'
    idx = np.array([9])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'n_FL'
    idx = np.array([10])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'n_FR'
    idx = np.array([11])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'v_x_pwm'
    idx = np.array([12])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'd_f_pwm'
    idx = np.array([13])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'what_x'
    idx = np.array([14])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'what_y'
    idx = np.array([15])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'what_z'
    idx = np.array([16])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

if __name__ == '__main__':
	try:
		rospy.wait_for_service('send_data')
		state_estimation()
	except rospy.ROSInterruptException:
		pass
