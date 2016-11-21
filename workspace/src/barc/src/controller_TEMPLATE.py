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
from barc.msg import Encoder, Ultrasound, ECU, Z_KinBkMdl
from math import pi,sin
import time
import serial
from numpy import zeros, hstack, cos, array, dot, arctan, eye
from system_models import f_KinBkMdl, h_KinBkMdl
from observers import kinematicLuembergerObserver, ekf
from system_models import f_KinBkMdl, h_KinBkMdl
from input_map import angle_2_servo, servo_2_angle
from pid import PID

###########################################################
# Set up measure callbacks
# from encoder
v 	        = 0
t0 	        = time.time()
n_FL	    = 0                     # counts in the front left tire
n_FR 	    = 0                     # counts in the front right tire
n_FL_prev 	= 0
n_FR_prev 	= 0
r_tire 		= 0.04                  # radius from tire center to perimeter along magnets [m]
dx_qrt 	    = 2.0*pi*r_tire/4.0     # distance along quarter tire edge [m]


# imu measurement update
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)
def imu_callback(data):
	global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
	(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = data.value

# encoder measurement update
enc_FL, enc_FR, enc_BL, enc_BR = zeros(4)
def enc_callback(data):
	global v, d_f, t0
	global n_FL, n_FR, n_FL_prev, n_FR_prev

	n_FL = data.FL
	n_FR = data.FR

	# compute time elapsed
	tf = time.time()
	dt = tf - t0
	
	# if enough time elapse has elapsed, estimate v_x
	dt_min = 0.20
	if dt >= dt_min:
		# compute speed :  speed = distance / time
		v_FL = float(n_FL- n_FL_prev)*dx_qrt/dt
		v_FR = float(n_FR- n_FR_prev)*dx_qrt/dt
		v 	 = (v_FL + v_FR)/2.0

		# update old data
		n_FL_prev   = n_FL
		n_FR_prev   = n_FR
		t0 	        = time.time()

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
    rospy.Subscriber('encoder', Encoder, enc_callback)
    rospy.Subscriber('ultrasound', Ultrasound, ultrasound_callback)
    nh = rospy.Publisher('ecu', ECU, queue_size = 10)
    state_pub 	= rospy.Publisher('state_estimate', Z_KinBkMdl, queue_size = 10)


	# set node rate
    rateHz  = 50
    rate 	= rospy.Rate(rateHz)
    dt      = 1.0 / rateHz
    pid     = PID(P=0.01193, I=1.193, D=0)
    
    #L = rospy.get_param('controller/L')
    #R = rospy.get_param('controller/R')
    #vx_des = rospy.get_param('controller/vx_des')
    L = 0.26
    Rdes = 1
    vxprev = 0
    vyprev = 0
    count = 0

    # get EKF observer properties
    q_std = 0.1             # std of process noise
    r_std = 0.1             # std of measurementnoise
    lf = 0.136525
    lr = 0.123475
    vhMdl = (lf,lr)
    yawprev = 0
    
    # estimation variables for Luemberger observer
    z_EKF       = zeros(4)

    # estimation variables for EKF
    P           = eye(4)                # initial dynamics coveriance matrix
    Q           = (q_std**2)*eye(4)     # process noise coveriance matrix
    R           = (r_std**2)*eye(2)     # measurement noise coveriance matrix


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

        # estimate actual and desired yaw rate
        psidot = w_z*(180/pi)
        psidotdes = (v/Rdes)*(180/pi)
        yawdes = yawprev + (v/Rdes)*dt

        # pid control steering angle
        e1 = psidot - psidotdes
        d_f = pid.update(e1, dt)
        if d_f > 30:
          d_f = 30
        elif d_f < -30:
          d_f = -30
        servo_PWM = angle_2_servo(d_f)

        motor_PWM = 97
        ecu_cmd = ECU(motor_PWM, servo_PWM)
        nh.publish(ecu_cmd)
        
        count += dt
        yawprev = yawdes
        # wait
        rate.sleep()

#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except rospy.ROSInterruptException:
		pass
