#!/usr/bin/env python

from __future__ import division
import rospy
from data_service.msg import TimeData
from barc.msg import ECU, Encoder
import time
import serial
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Image
from numpy import zeros, hstack, cos, array, dot, arctan
from math import sin, radians, atan2, degrees, sqrt, atan, pi
from input_map import angle_2_servo, servo_2_angle # pwm signals

def offset_callback(data): # This stores the current and previous values of lane offset in pixels 
	global pixel_offset 
	if data is not None:
		pixel_offset = data.data
	
# Speed estimator global variables
n_FL, n_FR, n_FL_prev, n_FR_prev = zeros(4) # using only front i.e steering tires
t0 = time.time()
v_x_curr = 0.0
v_x_prev = 0.0
d_f = 0.0
r_tire = 0.0319
dx_magnets = 2 * pi * r_tire // 4.0 # distance between two consecutif magnets 

def enc_callback(data):
	global v_x_curr, v_x_prev, d_f, t0, dx_magnets, n_FL, n_FR, n_FL_prev, n_FR_prev
	
	if data is not None:
		n_FL = data.FL
		n_FR = data.FR
			
  	tf = time.time() # Current Time
	dt = tf - t0 # time elapsed
	dt_min = 0.2 # set a minimum time of 0.2s after which we determine the speed of the vehicle
	
	if dt > dt_min:
		v_FL = float(n_FL - n_FL_prev) * dx_magnets / dt # using v = d / t
		v_FR = float(n_FR - n_FR_prev) * dx_magnets / dt 
		v_x_curr = (v_FL + v_FR) /2 * cos(radians(d_f)) # update encoder measurements for v_x and v_y
		v_x_curr = 0.75 * v_x_curr + 0.25 * v_x_prev # apply filtering to smooth velocity
		v_x_prev = v_x_curr # set current to previous for next iteration
		n_FL_prev = n_FL
		n_FR_prev = n_FR # update data
		t0 = time.time()
		
# Global steering Control variable
e_prev = 0
ei = 0.0
pixel_offset = 0.0


def steering_command(rateHz, Kp, Kd, Ki):
	global pixel_offset, e_prev, ei, d_f
	
	e = pixel_offset
	ed = (e - e_prev) * rateHz
	ei = ei + ( e / float(rateHz) )
	
	turn = (e * Kp) + (ei * Ki) + (ed * Kd)
	
	if (turn < -30.0):
		turn = -30.0
	elif (turn > 30.0):
		turn = 30.0
	d_f = turn
	e_prev = e
	return angle_2_servo(-turn), -turn	
		
		
		
		
# Global Speed Control Variables
max_torque = 0
eiS = 0.0

def speed_command(speed_desired, rateHz, KpS, KdS, KiS):
	global eiS, v_x_curr, max_torque, v_x_prev
	
	# Calcualte the error
	e = speed_desired - v_x_curr
	eSprev = speed_desired - v_x_prev
	ed = (e - eSprev) * rateHz
	eiS = eiS + (e / float(rateHz))		
		
	# Calculate Torque
	torque = 94.5 + ((e * KpS) + (ed * KdS) + (eiS * KiS)) # PID
	
	# Apply saturation
	if (torque > max_torque):
		torque = max_torque
	elif (torque < 90):
		torque = 90
	return torque
	
nh = None

##################################################################################################

def main_auto():
	global nh, v_x_curr, max_torque
	
	# initialize ROS node
	rospy.init_node('auto_mode', anonymous = True)
	nh = rospy.Publisher('ecu', ECU, queue_size = 10)
	steering_offset_subscriber = rospy.Subscriber('lane_offset', Float32, offset_callback)
	rospy.Subscriber('encoder', Encoder, enc_callback)
	
	# DATA logging
	torque_pub = rospy.Publisher('torque', Float32, queue_size = 10)
	steering_angle_pub = rospy.Publisher('steering_angle', Float32, queue_size = 10)
	speed_pub = rospy.Publisher('speed', Float32, queue_size = 10)
	
	# set Node Rate
	rateHz = 30
	rate = rospy.Rate(rateHz)
	
	# get Desired Speed
	speed_desired = rospy.get_param('speed_controller/speed_desired')
	max_torque = rospy.get_param('speed_controller/max_torque')
	
	# get speed Controller Parameters P I D
	KpS = rospy.get_param('speed_controller/KpS')
	KdS = rospy.get_param('speed_controller/KdS')
	KiS = rospy.get_param('speed_controller/KiS')
	
	# Get test time
	tf = rospy.get_param('speed_controller/test_time')
	
	# Set starting time
	t0 = time.time()
	
	# here starts the main loop
	while not rospy.is_shutdown():
		speedCMD = speed_command(speed_desired, rateHz, KpS, KdS, KiS) # Get speed command signal
		if ((time.time() - t0) > tf):
			speedCMD = 40
			
		servoCMD, angle = steering_command(rateHz, Kp, Kd, Ki) # Get steering command signal
		ecu_cmd = ECU(speedCMD, servoCMD)
		nh.publish(ecu_cmd) # publish the commands sent to the ecu these are the values that correct the car's position
		
		# Publish data
		torque_pub.publish(speedCMD)
		steering_angle_pub.publish(angle)
		speed_pub.publish(v_x_curr)
		
		# Wait
		rate.sleep()
		
###########################################################################################################
if __name__ == '__main__':
	try:
		main_auto()
	except rospy.ROSInterruptException:
		pass	
	

	
	
	
		
		
		
		
		
		
		
		
		
		
		