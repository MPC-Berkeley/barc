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
from data_service.msg import TimeData
from barc.msg import ECU
from math import pi,sin
import time
import serial
from numpy import genfromtxt, zeros, hstack, cos, array, dot, arctan
from input_map import angle_2_servo, servo_2_angle

def rc_inputs_callback(data):
    global throttle, steering
    throttle = data.motor_pwm
    steering = data.servo_pwm

def main_auto():
    global throttle, steering
    # initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    rospy.Subscriber('rc_inputs', ECU, rc_inputs_callback)
    nh = rospy.Publisher('ecu', ECU, queue_size = 10)

    # set node rate
    rateHz  = 50
    rate    = rospy.Rate(rateHz)

    throttle = 90
    steering = 90

    # main loop
    while not rospy.is_shutdown():

        # send command signal
        ecu_cmd = ECU(throttle, steering)
        nh.publish(ecu_cmd)

        rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        main_auto()
    except rospy.ROSInterruptException:
        pass
