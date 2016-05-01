#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

import rospy
import time
import os
from barc.msg import ECU, Encoder, Z_KinBkMdl
from data_service.msg import TimeData
from numpy import pi, cos, sin, eye, array, zeros, unwrap
from input_map import angle_2_servo, servo_2_angle
from observers import kinematicLuembergerObserver, ekf
from system_models import f_KinBkMdl, h_KinBkMdl
from filtering import filteredSignal
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

# input variables [default values]
d_f 	    = 0         # steering angle [deg]
a           = 0         # acceleration [m/s]
servo_pwm   = 90
motor_pwm   = 90

# raw measurement variables
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)
yaw_prev    = 0
psi         = 0

# from encoder
v 	        = 0
t0 	        = time.time()
n_FL	    = 0                     # counts in the front left tire
n_FR 	    = 0                     # counts in the front right tire
n_FL_prev 	= 0
n_FR_prev 	= 0
r_tire 		= 0.04                  # radius from tire center to perimeter along magnets [m]
dx_qrt 	    = 2.0*pi*r_tire/4.0     # distance along quarter tire edge [m]

# ecu command update
def ecu_callback(data):
    global motor_pwm, servo_pwm, d_f, a
    motor_pwm	    = data.motor_pwm
    servo_pwm       = data.servo_pwm
    d_f             = servo_2_angle(servo_pwm) * pi/180         # [rad]

    # apply acceleration input
    if a > 95:
        a           = 0.3*(motor_pwm - 95)                          # TODO: need to build correct mapping
    else:
        a           = 0

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
    global yaw_prev, psi
    (roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = data.value
    # unwrap angle measurements, since measurements wrap at plus/minus pi
    yaw         = unwrap(array([yaw_prev, yaw]), discont = 2*pi)[1]
    yaw_prev    = yaw
    psi         = yaw

# encoder measurement update
def enc_callback(data):
	global v, d_f, t0
	global n_FL, n_FR, n_FL_prev, n_FR_prev

	n_FL = data.FL
	n_FR = data.FR

	# compute time elapsed
	tf = time.time()
	dt = tf - t0
	
	# if enough time elapse has elapsed, estimate v_x
	dt_min = 0.20
	if dt >= dt_min:
		# compute speed :  speed = distance / time
		v_FL = float(n_FL- n_FL_prev)*dx_qrt/dt
		v_FR = float(n_FR- n_FR_prev)*dx_qrt/dt
		v_avg 	= (v_FL + v_FR)/2.0

		# update old data
		n_FL_prev   = n_FL
		n_FR_prev   = n_FR
		t0 	        = time.time()


# state estimation node
def state_estimation():
	# initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu', TimeData, imu_callback)
    rospy.Subscriber('encoder', Encoder, enc_callback)
    rospy.Subscriber('ecu', ECU, ecu_callback)
    state_pub 	= rospy.Publisher('state_estimate', Z_KinBkMdl, queue_size = 10)

	# get vehicle dimension parameters
    L_a = rospy.get_param("state_estimation/L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("state_estimation/L_b")       # distance from CoG to rear axel
    vhMdl   = (L_a, L_b)

    # get encoder parameters
    dt_vx   = rospy.get_param("state_estimation/dt_vx")     # time interval to compute v_x

    # get EKF observer properties
    q_std   = rospy.get_param("state_estimation/q_std")             # std of process noise
    r_std   = rospy.get_param("state_estimation/r_std")             # std of measurementnoise

	# set node rate
    loop_rate 	= 50
    dt 		    = 1.0 / loop_rate
    rate 		= rospy.Rate(loop_rate)
    t0 			= time.time()

    # estimation variables for Luemberger observer
    z_EKF       = zeros(4) 

    # estimation variables for EKF
    P           = eye(4)                # initial dynamics coveriance matrix
    Q           = (q_std**2)*eye(4)     # process noise coveriance matrix
    R           = (r_std**2)*eye(2)     # measurement noise coveriance matrix

    while not rospy.is_shutdown():

		# publish state estimate
        (x, y, psi, v) = z_EKF          

        # publish information
        state_pub.publish( Z_KinBkMdl(x, y, psi, v) )

        # collect measurements, inputs, system properties
        # collect inputs
        y   = array([psi, v])
        u   = array([ d_f, a ])
        args = (u,vhMdl,dt) 

        # apply EKF and get each state estimate
        (z_EKF,P) = ekf(f_KinBkMdl, z_EKF, P, h_KinBkMdl, y, Q, R, args )

		# wait
        rate.sleep()

if __name__ == '__main__':
	try:
	   state_estimation()
	except rospy.ROSInterruptException:
		pass
