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

#############################################################
# get estimate of x_hat = [v_x , v_y, w_z]
# x_hat = zeros(3)
# def updateState_callback(data):
#   global x_hat

#   # update fields
#   x_hat[0] = data.x       # v_x  longitudinal velocity
#   x_hat[1] = data.y       # v_y  lateral velocity
#   x_hat[2] = data.z       # w_z  angular velocity about z-axis

rc_inputs = (90, 90)
def receiveRC_callback(data):
    global rc_inputs
    rc_inputs[0] = data.x #throttle
    rc_inputs[1] = data.y #steering
    rospy.loginfo("motor: ")
    rospy.loginfo("%d", data.x)
    rospy.loginfo("servo: ")
    rospy.loginfo("%d", data.y)

#############################################################
def main_auto():
    # initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    # rospy.Subscriber('state_estimate', Vector3, updateState_callback)
    rospy.Subscriber('rc_cmd', Vector3, receiveRC_callback)
    nh = rospy.Publisher('ecu_cmd', Vector3, queue_size = 10)

    # set node rate
    rateHz  = 50
    rate    = rospy.Rate(rateHz)
    t_i     = 0

    # get node parameters
    timeout = rospy.get_param("controller/runLength")

    # main loop
    while not rospy.is_shutdown():
        # get command signal
        (motorCMD, servoCMD) = (rc_inputs[0], rc_inputs[1])

        # send command signal
        ecu_cmd = Vector3(motorCMD, servoCMD, 0)
        # TODO uncomment once this is verified as OK
        # nh.publish(ecu_cmd)

        # wait
        t_i += 1
        rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        main_auto()
    except rospy.ROSInterruptException:
        pass
