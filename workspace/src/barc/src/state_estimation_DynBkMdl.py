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
import time
import os
from barc.msg import ECU, Encoder
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from numpy import pi, cos, sin, eye, array, zeros
from ekf import ekf
from system_models import f_3s, h_3s
from tf import transformations
from numpy import unwrap

# input variables
d_f 	    = 0
FxR         = 0

# raw measurement variables
yaw_prev = 0
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)

# from encoder
v_x_enc 	= 0
t0 	        = time.time()
n_FL	    = 0                     # counts in the front left tire
n_FR 	    = 0                     # counts in the front right tire
n_FL_prev 	= 0
n_FR_prev 	= 0
r_tire 		= 0.04                  # radius from tire center to perimeter along magnets [m]
dx_qrt 	    = 2.0*pi*r_tire/4.0     # distance along quarter tire edge

# ecu command update
def ecu_callback(data):
    global FxR, d_f
    FxR         = data.motor        # input motor force [N]
    d_f         = data.servo        # input steering angle [rad]

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
    global yaw_prev
    
    # get orientation from quaternion data, and convert to roll, pitch, yaw
    # extract angular velocity and linear acceleration data
    ori  = data.orientation
    quaternion  = (ori.x, ori.y, ori.z, ori.w)
    (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)
    yaw         = unwrap(array([yaw_prev, yaw]), discont = pi)[1]
    yaw_prev    = yaw
    
    # extract angular velocity and linear acceleration data
    w_x = data.angular_velocity.x
    w_y = data.angular_velocity.y
    w_z = data.angular_velocity.z
    a_x = data.linear_acceleration.x
    a_y = data.linear_acceleration.y
    a_z = data.linear_acceleration.z

# encoder measurement update
def enc_callback(data):
	global v_x_enc, d_f, t0
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

		# update encoder v_x, v_y measurements
		# only valid for small slip angles, still valid for drift?
		v_x_enc 	= (v_FL + v_FR)/2.0*cos(d_f)

		# update old data
		n_FL_prev   = n_FL
		n_FR_prev   = n_FR
		t0 	        = time.time()


# state estimation node
def state_estimation():
	# initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu/data', Imu, imu_callback)
    rospy.Subscriber('encoder', Encoder, enc_callback)
    rospy.Subscriber('ecu', ECU, ecu_callback)
    state_pub 	= rospy.Publisher('state_estimate', Vector3, queue_size = 10)

	# get vehicle dimension parameters
    L_a = rospy.get_param("L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("L_b")       # distance from CoG to rear axel
    m   = rospy.get_param("m")         # mass of vehicle
    I_z = rospy.get_param("I_z")       # moment of inertia about z-axis
    vhMdl   = (L_a, L_b, m, I_z)

    # get encoder parameters
    dt_vx   = rospy.get_param("state_estimation/dt_v_enc")     # time interval to compute v_x

    # get tire model
    B   = rospy.get_param("tire_model/B")
    C   = rospy.get_param("tire_model/C")
    mu  = rospy.get_param("tire_model/mu")
    TrMdl = ([B,C,mu],[B,C,mu])

    # get external force model
    a0  = rospy.get_param("air_drag_coeff")
    Ff  = rospy.get_param("friction")

    # get EKF observer properties
    q_std   = rospy.get_param("state_estimation/q_std")             # std of process noise
    r_std   = rospy.get_param("state_estimation/r_std")             # std of measurementnoise
    v_x_min     = rospy.get_param("state_estimation/v_x_min")  # minimum velociy before using EKF

	# set node rate
    loop_rate 	= 50
    dt 		    = 1.0 / loop_rate
    rate 		= rospy.Rate(loop_rate)
    t0 			= time.time()

    # estimation variables for Luemberger observer
    z_EKF       = array([1.0, 0.0, 0.0])

    # estimation variables for EKF
    P           = eye(3)                # initial dynamics coveriance matrix
    Q           = (q_std**2)*eye(3)     # process noise coveriance matrix
    R           = (r_std**2)*eye(2)     # measurement noise coveriance matrix

    while not rospy.is_shutdown():

		# publish state estimate
        (v_x, v_y, r) = z_EKF           # note, r = EKF estimate yaw rate

        # publish information
        state_pub.publish( Vector3(v_x, v_y, r) )

        # apply EKF
        if v_x_enc > v_x_min:
            # get measurement
            y = array([v_x_enc, w_z])

            # define input
            u       = array([ d_f, FxR ])

            # build extra arguments for non-linear function
            F_ext = array([ a0, Ff ]) 
            args = (u, vhMdl, TrMdl, F_ext, dt) 

            # apply EKF and get each state estimate
            (z_EKF,P) = ekf(f_3s, z_EKF, P, h_3s, y, Q, R, args )

        else:
            z_EKF[0] = float(v_x_enc)
            z_EKF[2] = float(w_z)
        
		# wait
        rate.sleep()

if __name__ == '__main__':
	try:
	   state_estimation()
	except rospy.ROSInterruptException:
		pass
