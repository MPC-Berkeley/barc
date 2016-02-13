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
from geometry_msgs.msg import Vector3, Twist
from data_service.msg import TimeData
from math import pi,sin
import time
import serial
from numpy import genfromtxt, zeros, hstack, cos, array, dot, arctan
from input_map import angle_2_servo, servo_2_angle
from manuevers import TestSettings, CircularTest, Straight, SineSweep, DoubleLaneChange

#############################################################
# get estimate of x_hat = [v_x , v_y, w_z]
x_hat = zeros(3)
def updateState_callback(data):
	global x_hat

	# update fields
	x_hat[0] = data.x 		# v_x  longitudinal velocity 
	x_hat[1] = data.y 		# v_y  lateral velocity
	x_hat[2] = data.z		# w_z  angular velocity about z-axis

#############################################################
def LQR_drift(test_opt, init_sequence, K_LQR, rate, t_i):
	# initial setting
	oneSecCount = rate
	N 			= init_sequence.size 	# length of opening sequence
	dt 			= 7*oneSecCount

	t_0			= 5*oneSecCount
	t_OL 		= t_0 + N
	t_f 		= t_OL + dt

	# get current state estimate
	global x_hat
	v_x 	= x_hat[0]
	v_y 	= x_hat[1]
	w_z 	= x_hat[2]

	# compute slip angle beta
	if v_x == 0:
		beta = 0
	else:
		beta  	= arctan(v_y/v_x)

	# define lqr state [beta, w_z, v_x]
	x_lqr 	= array([beta, w_z, v_x])

	# OPEN LOOP CONTROL 
	# get to speed, then apply open loop steering sequence
	if t_i < t_0:
		servoCMD  	= test_opt.Z_turn
		motorCMD 	= test_opt.speed

	# start
	elif t_i < t_OL:
		k 		 = t_i - t_0
		d_f 	 = init_sequence[k] * 180/pi
		servoCMD  = angle_2_servo(d_f) 
		motorCMD = test_opt.speed

	# FEEDBACK CONTROL
	elif t_i < t_f:
		# compute LQR gain
		u 		= dot(K_LQR, x_lqr)

		# extract individual inputs
		d_f 		= u[0]*180/pi	
		v_x_ref 	= u[1]
		
		# TO DO -- NEED MAPPING FROM v_x_ref to ESC PWM signal
		servoCMD  = angle_2_servo(d_f) 
		motorCMD = test_opt.speed
	
    # set straight and stop
	else:
		servoCMD     	= test_opt.Z_turn
		motorCMD        = test_opt.neutral
		if not test_opt.stopped:
			motorCMD    	 = test_opt.brake
			test_opt.stopped = True

	return (motorCMD, servoCMD)

#############################################################
def main_auto():
	# initialize ROS node
	rospy.init_node('auto_mode', anonymous=True)
	rospy.Subscriber('state_estimate', Vector3, updateState_callback)
	nh = rospy.Publisher('ecu_cmd', Vector3, queue_size = 10)

	# set node rate
	rateHz  = 50
	rate 	= rospy.Rate(rateHz)
	t_i     = 0

	# get initial OL sequence for feedback control
	# open loop drift maneuver - driver straigh, then perform manuever to enter drift
	dir_path = '/home/odroid/catkin_ws/src/barc/data'
	start_drift_maneuver 	= genfromtxt(dir_path + '/startDriftManeuver',delimiter=',')
	drive_straight 			= zeros(rateHz * 1)
	initial_sequence 		= hstack((drive_straight, start_drift_maneuver)) 

	# LQR gain
	K_LQR = genfromtxt(dir_path + '/K_lqr', delimiter=',') 
    test_opt = TestSettings()

	while not rospy.is_shutdown():
		# get command signal
		(motorCMD, servoCMD) = LQR_drift(test_opt, initial_sequence, K_LQR, rateHz, t_i)
			
        # send command signal 
		ecu_cmd = Vector3(motorCMD, servoCMD, 0)
		nh.publish(ecu_cmd)
	
        # wait
		t_i += 1
		rate.sleep()

#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except rospy.ROSInterruptException:
		pass
