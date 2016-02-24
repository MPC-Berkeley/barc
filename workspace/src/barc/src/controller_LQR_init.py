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
from manuevers import TestSettings, CircularTest, Straight
from manuevers import SineSweep, DoubleLaneChange, CoastDown, SingleTurn 

#############################################################
def main_auto():
    # initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    nh = rospy.Publisher('ecu_cmd', Vector3, queue_size = 10)

	# set node rate
    rateHz  = 50
    rate 	= rospy.Rate(rateHz)
    t_i     = 0

	# get node parameters
    experiment_sel 	= rospy.get_param("controller/experiment_sel")
    v_x_pwm 	= rospy.get_param("controller/v_x_pwm")
    t_exp 		= rospy.get_param("controller/t_exp")
    t_turn      = rospy.get_param("controller/t_turn")

    # specify test and test options
    experiment_opt    = { 0 : CircularTest,
                          1 : Straight,
		    		      2 : SineSweep,   
                          3 : DoubleLaneChange,
					      4 : CoastDown ,
					      5 : SingleTurn}
    test_mode   = experiment_opt.get(experiment_sel)
    str_ang 	= rospy.get_param("controller/steering_angle")
    test_opt 	= TestSettings(SPD = v_x_pwm, turn = str_ang, dt=t_exp)
    test_opt.t_turn = t_turn
	

    # main loop
    while not rospy.is_shutdown():
        # get command signal
        (motorCMD, servoCMD) = test_mode(test_opt, rateHz, t_i)
			
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
