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
from data_service.msg import TimeData
from barc.msg import ECU
from math import pi,sin
import time
import serial
from numpy import zeros, hstack, cos, array, dot, arctan
from input_map import angle_2_servo, servo_2_angle
from manuevers import TestSettings, CircularTest, Straight, SineSweep, DoubleLaneChange, CoastDown

#############################################################
def main_auto():
    # initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    nh = rospy.Publisher('ecu', ECU, queue_size = 10)

	# set node rate
    rateHz  = 50
    rate 	= rospy.Rate(rateHz)
    t_i     = 0

    # specify test and test options
    experiment_opt    = { 0 : CircularTest,
                          1 : Straight,
		    		      2 : SineSweep,   
                          3 : DoubleLaneChange,
					      4 : CoastDown }
    experiment_sel 	= rospy.get_param("controller/experiment_sel")
    v_x_pwm 	= rospy.get_param("controller/v_x_pwm")
    t_0         = rospy.get_param("controller/t_0")
    t_exp 		= rospy.get_param("controller/t_exp")
    str_ang 	= rospy.get_param("controller/steering_angle")
    test_mode   = experiment_opt.get(experiment_sel)
    opt 	    = TestSettings(SPD = v_x_pwm, turn = str_ang, dt=t_exp)
    opt.t_0    = t_0
	

    # main loop
    while not rospy.is_shutdown():
        # get command signal
        (motorCMD, servoCMD) = test_mode(opt, rateHz, t_i)
			
        # send command signal 
        ecu_cmd = ECU(motorCMD, servoCMD)
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
