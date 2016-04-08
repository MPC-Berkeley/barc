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
from manuevers import TestSettings, CircularTest, Straight, SineSweep, DoubleLaneChange, CoastDown
from pid import PID


# pid control for constrant yaw angle 
# -> d(yaw)/dt = 0 means no turning => straight path
yaw0    = 0                 # counts in the front left tire
yaw     = 0
read_yaw0 = False
w_z 	  = 0                 # counts in the front right tire
err     = 0
def angle_callback(data):
    global yaw, yaw0, w_z, err, read_yaw0

    # save initial measurements
    if not read_yaw0:
        yaw0 = data.x
        read_yaw0 = True

    # compute deviation from original yaw angle
    yaw = data.x
    err = yaw - yaw0

#############################################################
def main_auto():
    # initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    nh = rospy.Publisher('ecu', Vector3, queue_size = 10)
    rospy.Subscriber('angle_info', Vector3, angle_callback)

	# set node rate
    rateHz  = 50
    dt      = 1.0 / rateHz
    rate 	= rospy.Rate(rateHz)
    t_i     = 0

    # specify test and test options
    experiment_opt    = { 0 : CircularTest,
                          1 : Straight,
		    		      2 : SineSweep,   
                          3 : DoubleLaneChange,
					      4 : CoastDown}
    experiment_sel 	= rospy.get_param("controller/experiment_sel")
    v_x_pwm 	= rospy.get_param("controller/v_x_pwm")
    t_0         = rospy.get_param("controller/t_0")
    t_exp 		= rospy.get_param("controller/t_exp")
    str_ang 	= rospy.get_param("controller/steering_angle")
    test_mode   = experiment_opt.get(experiment_sel)
    opt 	    = TestSettings(SPD = v_x_pwm, turn = str_ang, dt=t_exp)
    opt.t_0    = t_0
	
    # use simple pid control to keep steering straight
    p 		= rospy.get_param("controller/p")
    i 		= rospy.get_param("controller/i")
    d 		= rospy.get_param("controller/d")
    pid     = PID(P=p, I=i, D=d)

    # main loop
    while not rospy.is_shutdown():
        # get steering wheel command
        global err
        u         = pid.update(err, dt)
        servoCMD  = angle_2_servo(u)
        
        # get command signal
        (motorCMD, _) = test_mode(opt, rateHz, t_i)
			
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
