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
from numpy import genfromtxt, zeros, hstack, cos, array, dot, arctan
from input_map import angle_2_servo, servo_2_angle

def rc_inputs_callback(data):
    global throttle, steering
    throttle = data.motor_pwm
    steering = data.servo_pwm

def enc_callback(data):
    global n_FL, n_FR, n_FL_prev, n_BR_prev, n_BL_prev, slip

    n_FL = data.FL
    n_BR = data.BR
    n_BL = data.BL

    # Maintaining the last five (or however many specified at top of main loop)
    # readings
    n_FL_prev.pop()
    n_BR_prev.pop()
    n_BL_prev.pop()
    n_FL_prev.append(n_FL)
    n_BR_prev.append(n_BR)
    n_BL_prev.append(n_BL)

    n_FL_diff = n_FL_prev[4] - n_FL_prev[0]
    n_BR_diff = n_BR_prev[4] - n_BR_prev[0]
    n_BL_diff = n_BL_prev[4] - n_BL_prev[0]
    # could later determine which wheel is slipping more to add corrective
    # steering toward stronger wheel
    B_diff = (n_BR_diff + n_BL_diff)/2

    if n_FL_diff == 0:
        F_diff = 1
    else:
        F_diff = n_FL_diff

    slip = (B_diff - F_diff)/float(F_diff)
    # maybe undo this, could be decent for ABS?
    if slip < 0:
        slip = 0

def main_auto():
    global n_FL_prev, n_BR_prev, n_BL_prev
    global throttle, steering
    global slip
    n_FL_prev = [0, 0, 0, 0, 0]
    n_BR_prev = [0, 0, 0, 0, 0]
    n_BL_prev = [0, 0, 0, 0, 0]
    # initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    rospy.Subscriber('rc_inputs', ECU, rc_inputs_callback)
    rospy.Subscriber('encoder', Encoder, enc_callback)
    nh = rospy.Publisher('ecu', ECU, queue_size = 10)

    # set node rate
    rateHz  = 50
    rate    = rospy.Rate(rateHz)

    throttle = 90
    steering = 90

    kP = 5
    slip = 0

    # main loop
    while not rospy.is_shutdown():

        # send command signal
        pThrottle = int(throttle - kP*slip)
        if pThrottle < 90:
            # if throttle > 95:
            #     pThrottle = 95
            # else:
            #     pThrottle = 90
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
