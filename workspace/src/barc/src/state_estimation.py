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
from numpy import pi, cos, sin, eye, array
from geometry_msgs.msg import Vector3
from input_map import angle_2_servo, servo_2_angle
from observers import kinematicLuembergerObserver, ekf
from system_models import f_3s, h_3s
from data_service.srv import *
from data_service.msg import *
from filtering import filteredSignal

import numpy as np

# input variables
d_f 	    = 0
servo_pwm   = 0
motor_pwm   = 0

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
v_x_enc 	= 0
t0 	        = time.time()
n_FL	    = 0                 # counts in the front left tire
n_FR 	    = 0                 # counts in the front right tire
n_FL_prev 	= 0
n_FR_prev 	= 0
r_tire 		= 0.0319            # radius from tire center to perimeter along magnets
dx_magnets 	= 2.0*pi*r_tire/4.0     # distance between magnets

# ecu command update
def ecu_callback(data):
	global servo_pwm, motor_pwm, d_f
	motor_pwm	= data.x
	servo_pwm = data.y
	d_f 		= pi/180.0*servo_2_angle(servo_pwm)

# imu measurement update
def imu_callback(data):
	global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
	(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = data.value

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
	if dt >= dt_min:
		# compute speed :  speed = distance / time
		v_FL = float(n_FL- n_FL_prev)*dx_magnets/dt
		v_FR = float(n_FR- n_FR_prev)*dx_magnets/dt

		# update encoder v_x, v_y measurements
		# only valid for small slip angles, still valid for drift?
		v_x_enc 	= (v_FL + v_FR)/2.0*cos(d_f)

		# update old data
		n_FL_prev   = n_FL
		n_FR_prev   = n_FR
		t0 	        = time.time()


# state estimation node
def state_estimation():
	  # initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu_data', TimeData, imu_callback)
    rospy.Subscriber('enc_data', Vector3, enc_callback)
    rospy.Subscriber('ecu_cmd', Vector3, ecu_callback)
    state_pub 	= rospy.Publisher('state_estimate', Vector3, queue_size = 10)
    angle_pub 	= rospy.Publisher('angle_info', Vector3, queue_size = 10)

	  # get system parameters
    username = rospy.get_param("controller/user")
    experiment_sel 	= rospy.get_param("controller/experiment_sel")
    experiment_opt 	= {0 : "Circular",
				       1 : "Straight",
				       2 : "SineSweep",
				       3 : "DoubleLaneChange",
				       4 : "CoastDown",
				       5 : "SingleTurn"}

    experiment_type = experiment_opt.get(experiment_sel)
    signal_ID = username + "-" + experiment_type
    experiment_name = signal_ID
    
	  # get vehicle dimension parameters
    # note, the imu is installed at the front axel
    L_a = rospy.get_param("state_estimation/L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("state_estimation/L_b")       # distance from CoG to rear axel
    m   = rospy.get_param("state_estimation/m")         # mass of vehicle
    I_z = rospy.get_param("state_estimation/I_z")       # moment of inertia about z-axis
    vhMdl   = (L_a, L_b, m, I_z)

    # get encoder parameters
    dt_vx   = rospy.get_param("state_estimation/dt_vx")     # time interval to compute v_x

    # get tire model
    B   = rospy.get_param("state_estimation/B")
    C   = rospy.get_param("state_estimation/C")
    mu  = rospy.get_param("state_estimation/mu")
    TrMdl = ([B,C,mu],[B,C,mu])

    # get external force model
    a0  = rospy.get_param("state_estimation/air_drag_coeff")
    Ff  = rospy.get_param("state_estimation/Ff")

    # get Luemberger and EKF observer properties
    aph = rospy.get_param("state_estimation/aph")             # parameter to tune estimation error dynamics
    q_std   = rospy.get_param("state_estimation/q")             # std of process noise
    r_std   = rospy.get_param("state_estimation/r")             # std of measurementnoise
    v_x_min     = rospy.get_param("state_estimation/v_x_min")  # minimum velociy before using EKF

	  # set node rate
    loop_rate 	= 50
    dt 		    = 1.0 / loop_rate
    rate 		= rospy.Rate(loop_rate)

    ## Open file to save data
    date 				= time.strftime("%Y.%m.%d")
    BASE_PATH   		= "/home/odroid/Data/" + date + "/"
	  # create directory if it doesn't exist
    if not os.path.exists(BASE_PATH):
        os.makedirs(BASE_PATH)
    data_file_name   	= BASE_PATH + signal_ID + '-' + time.strftime("%H.%M.%S") + '.csv'
    data_file     		= open(data_file_name, 'a')
    data_file.write('t,roll,pitch,yaw,w_x,w_y,w_z,a_x,a_y,a_z,n_FL,n_FR,motor_pwm,servo_pwm,d_f,vhat_x,vhat_y,what_z,v_x_enc\n')
    t0 				= time.time()

    # estimation variables for Luemberger observer
    # z_EKF = [v_x, v_y, w_z]
    z_EKF       = array([1.0, 0.0, 0.0])

    # estimation variables for EKF
    P           = eye(3)                # initial dynamics coveriance matrix
    Q           = (q_std**2)*eye(3)     # process noise coveriance matrix
    R           = (r_std**2)*eye(2)     # measurement noise coveriance matrix

    # filtered signal for longitudinal velocity
    p_filter    = rospy.get_param("state_estimation/p_filter")
    v_x_filt    = filteredSignal(a = p_filter, method='lp')   # low pass filter

    while not rospy.is_shutdown():
	    # signals from inertial measurement unit, encoder, and control module
        global roll, pitch, yaw, w_x, w_y, w_z, a_x, a_y, a_z
        global n_FL, n_FR, v_x_enc
        global motor_pwm, servo_pwm, d_f

		# publish state estimate
        (v_x, v_y, r) = z_EKF           # note, r = EKF estimate yaw rate

        # publish information
        state_pub.publish( Vector3(v_x, v_y, r) )
        angle_pub.publish( Vector3(yaw, w_z, 0) )

		# save data (maybe should use rosbag in the future)
        t  	= time.time() - t0
        all_data = [t,roll,pitch,yaw,w_x,w_y,w_z,a_x,a_y,a_z,n_FL,n_FR,motor_pwm,servo_pwm,d_f,v_x,v_y,r,v_x_enc]

        # save to CSV
        N = len(all_data)
        str_fmt = '%.4f,'*N
        data_file.write( (str_fmt[0:-1]+'\n') % tuple(all_data))

        # update filtered signal
        v_x_filt.update(v_x_enc)
        v_x_est = v_x_filt.getFilteredSignal() 


        # apply EKF
        if v_x_est > v_x_min:
            # get measurement
            y = array([v_x_est, w_z])

            # compute input motor signal 
            if motor_pwm >= 95:
                FxR     = (motor_pwm - 95)*0.3*m        # mapping from motor pwm to input force FxR
            else:
                FxR     = 0.0

            # define input
            u       = array([ d_f, FxR ])

            # build extra arguments for non-linear function
            F_ext = array([ a0, Ff ]) 
            args = (u, vhMdl, TrMdl, F_ext, dt) 

            # apply EKF and get each state estimate
            (z_EKF,P) = ekf(f_3s, z_EKF, P, h_3s, y, Q, R, args )

        else:
            z_EKF[0] = float(v_x_enc)
            z_EKF[2] = float(w_z)
        
		# wait
        rate.sleep()


# Reduce duplication somehow?
def send_all_data(data, timestamps, send_data, experiment_name):

    # print data
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

    time_signal.name = 'a_x'
    idx = np.array([7])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'a_y'
    idx = np.array([8])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'a_z'
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

    time_signal.name = 'motor_pwm'
    idx = np.array([12])
    time_signal.signal = json.dumps(data[:, idx].tolist())
    send_data(time_signal, None, experiment_name)

    time_signal.name = 'servo_pwm'
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
	  # rospy.wait_for_service('send_data')
	  state_estimation()
	except rospy.ROSInterruptException:
		pass
