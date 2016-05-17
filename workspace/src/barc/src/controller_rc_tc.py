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
from barc.msg import ECU, Encoder
from math import pi,sin
import time
import serial
import pandas as pd
from numpy import genfromtxt, zeros, hstack, cos, array, dot, arctan
from input_map import angle_2_servo, servo_2_angle
from pid import PID
from pddd import PDDD

def rc_inputs_callback(data):
    global throttle, steering
    throttle = data.motor_pwm
    steering = data.servo_pwm

def enc_callback(data):
    global n_FL, n_BR, n_BL

    n_FL = data.FL
    n_BR = data.BR
    n_BL = data.BL


def main_auto():
    global n_FL, n_BR, n_BL
    global throttle, steering
    n_FL = 0
    n_BR = 0
    n_BL = 0
    n_FL_prev = 0
    n_BR_prev = 0
    n_BL_prev = 0
    # initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    rospy.Subscriber('rc_inputs', ECU, rc_inputs_callback)
    rospy.Subscriber('encoder', Encoder, enc_callback)
    nh = rospy.Publisher('ecu', ECU, queue_size = 10)

    # set node rate
    rateHz  = 50
    dt = 1.0 /rateHz
    rate    = rospy.Rate(rateHz)

    throttle = 90
    steering = 90

    kP = 150
    slip = 0
    # pid = PID(P=0, I=0, D=5, Integrator_max=10, Integrator_min=0)
    pddd = PDDD(P=5, D=5, DD=0.05, set_point = 0.06, dt=dt, memory_time = 0.2)

    # main loop
    while not rospy.is_shutdown():
        # n_FL_diff = n_FL_prev - n_FL
        # n_BR_diff = n_BR_prev - n_BR
        # n_BL_diff = n_BL_prev - n_BL
        # B_diff = (n_BR_diff + n_BL_diff)/2

        # if n_FL_diff == 0:
        #     F_diff = 1
        # else:
        #     F_diff = n_FL_diff

        # slip = (B_diff - F_diff)/float(F_diff)
        # rospy.loginfo("n_FL: " + str(n_FL) + ", n_BR: " + str(n_BR) + ", n_BL: " + str(n_BL))
        # rospy.loginfo("n_FL_prev: " + str(n_FL_prev) + ", n_BR_prev: " + str(n_BR_prev) + ", n_BL_prev: " + str(n_BL_prev))
        # rospy.loginfo("Slip: " + str(slip) + ", B_diff: " + str(B_diff) + ", F_diff: " + str(F_diff))
        # maybe undo this, could be decent for ABS?
        if slip < 0:
            slip = 0

        n_FL_prev = n_FL
        n_BR_prev = n_BR
        n_BL_prev = n_BL

        # idea: controlling the error in encoder readings is the integral of
        # slip rate. So do a P-D-DD to effectively do a PID on slip rate
        # really though you want the integral to not just be limited by have
        # limited memory
        back = (n_BL + n_BR)/2
        u = pddd.update(n_FL, back, dt)
        # send command signal
        pThrottle = int(throttle + u)
        if throttle > 95:
            if pThrottle < 95:
                pThrottle = 95
        elif throttle < 88:
            pThrottle = throttle
        else:
            pThrottle = 90

        ecu_cmd = ECU(pThrottle, steering)
        nh.publish(ecu_cmd)

        rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        main_auto()
    except rospy.ROSInterruptException:
        pass
