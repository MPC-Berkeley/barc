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
from barc.msg import Encoder, Ultrasound, ECU
from math import pi,sin
import time
import serial
from numpy import zeros, hstack, cos, array, dot, arctan
from input_map import angle_2_servo, servo_2_angle
from pid import PID

###########################################################
# Set up measure callbacks
# imu measurement update
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)
def imu_callback(data):
	global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
	(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = data.value

# encoder measurement update
enc_FL, enc_FR, enc_BL, enc_BR = zeros(4)
def encoder_callback(data):
	global enc_FL, enc_FR, enc_BL, enc_BR
	enc_FL = data.FL
	enc_FR = data.FR
	enc_BL = data.BL
	enc_BR = data.BR

# ultrasound measurement update
(us_F, us_B, us_R, us_L) = zeros(4)
def ultrasound_callback(data):
    global us_F, us_B, us_R, us_L
    us_F = data.front
    us_B = data.back
    us_R = data.right
    us_L = data.left

#############################################################
# main code
def main_auto():
    # initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    rospy.Subscriber('imu', TimeData, imu_callback)
    rospy.Subscriber('encoder', Encoder, encoder_callback)
    rospy.Subscriber('ultrasound', Ultrasound, ultrasound_callback)
    nh = rospy.Publisher('ecu', ECU, queue_size = 10)

	# set node rate
    rateHz  = 50
    rate 	= rospy.Rate(rateHz)
    dt      = 1.0 / rateHz 
    pid     = PID(P=1, I=1, D=0)

    # main loop
    while not rospy.is_shutdown():
        """
        Using the following sensor measurements (i.e. system feedback) to design a controller

        From imu
        * roll  := roll angle [rad]
        * pitch := pitch angle [rad]
        * yaw   := yaw angle [rad]
        * w_x   := roll rate [rad/s]
        * w_y   := pitch rate [rad/s]
        * w_z   := yaw rate [rad/s]
        * a_x   := longitudinal acceleration in the imu coordinate frame [m/s^2]
        * a_y   := laterial acceleration in the imu coordinate frame [m/s^2]
        * a_z   := vertical accleration in the imu coordinate frame [m/s^2]
        
        From encoder
        ---- NOTE: at the moment, the arduino only use gives two encoder measurements
        ---- since it has only two interrupts (only front two give measurements)
        * enc_FL := number of counts from front left hall effect sensor
        * enc_FR := number of counts from front right hall effect sensor
        - enc_BL := number of counts from back left hall effect sensor
        - enc_BR := number of counts from back right hall effect sensor

        From ultrasound
        * us_F      := distance between front sensor and nearest solid object [cm]
        * us_B      := distance between back sensor and nearest solid object [cm]
        * us_L      := distance between front sensor and nearest solid object [cm]
        * us_R      := distance between front sensor and nearest solid object [cm]

        ---------------------------------------------------------------------------
        Example:
        * goal: Go straight
        * strategy: design a PID controller to keep 
                - encoder counts between two wheels equal
        
        ---------------------------------------------------------------------------

        # compute PID error for steering angle
        # convert desired steering angle [deg] into a PWM signal
        err             = enc_FL - enc_FR 
        steering_angle  = pid.update(err, dt)     # [deg]
        servo_PWM       = angle_2_servo(steering_angle)

        # set desired longitudinal input force
        # NOTE: the last expression depends on your vehicle model, this is just an example 
        m           = 1.85                      # mass of vehicle [kg]
        FxR         = 1                         # input force [N]
        motor_PWM   = FxR / (0.3*m) + 95        # mapping from FxR to motor PWM 
        
        ---------------------------------------------------------------------------
        """

        # publish command signal 
        motor_PWM   = 90
        servo_PWM   = 90
        ecu_cmd = ECU(motor_PWM, servo_PWM)
        nh.publish(ecu_cmd)
	
        # wait
        rate.sleep()

#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except rospy.ROSInterruptException:
		pass
