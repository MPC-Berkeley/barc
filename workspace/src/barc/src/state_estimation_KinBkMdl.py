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
from sensor_msgs.msg import Imu
from barc.msg import ECU, Encoder, Z_KinBkMdl, vel_sgn
from numpy import pi, cos, sin, eye, array, zeros, unwrap
from observers import kinematicLuembergerObserver, ekf
from system_models import f_KinBkMdl, h_KinBkMdl
from tf import transformations
from numpy import unwrap

# input variables [default values]
d_f         = 0         # steering angle [deg]
acc         = 0         # acceleration [m/s]

# velocity sign
sign = 1

# raw measurement variables
yaw_prev = 0
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)
yaw_prev    = 0
yaw_local   = 0
read_yaw0   = False
psi         = 0
psi_meas    = 0

# from encoder
v           = 0
v_meas      = 0
t0          = time.time()
n_FL        = 0                     # counts in the front left tire
n_FR        = 0                     # counts in the front right tire
n_BL        = 0                     # counts in the back left tire
n_BR        = 0                     # counts in the back right tire
n_FL_prev   = 0
n_FR_prev   = 0
n_BL_prev   = 0
n_BR_prev   = 0
r_tire      = 0.036                  # radius from tire center to perimeter along magnets [m]
dx_qrt      = 2.0*pi*r_tire/4.0     # distance along quarter tire edge [m]

def sign_callback(data):
    global sign
    sign = data.mode

# ecu command update
def ecu_callback(data):
    global acc, d_f
    acc         = data.motor        # input acceleration
    d_f         = data.servo        # input steering angle

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
    global yaw_prev, yaw0, read_yaw0, yaw_local
    global psi_meas

    # get orientation from quaternion data, and convert to roll, pitch, yaw
    # extract angular velocity and linear acceleration data
    ori         = data.orientation
    quaternion  = (ori.x, ori.y, ori.z, ori.w)
    (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)

    # save initial measurements
    if not read_yaw0:
        read_yaw0   = True
        yaw_prev    = yaw
        yaw0        = yaw
    
    # unwrap measurement
    yaw         = unwrap(array([yaw_prev, yaw]), discont = pi)[1]
    yaw_prev    = yaw
    yaw_local   = yaw - yaw0
    psi_meas    = yaw_local
    
    # extract angular velocity and linear acceleration data
    w_x = data.angular_velocity.x
    w_y = data.angular_velocity.y
    w_z = data.angular_velocity.z
    a_x = data.linear_acceleration.x
    a_y = data.linear_acceleration.y
    a_z = data.linear_acceleration.z

# encoder measurement update
def enc_callback(data):
    global v, t0, dt_v_enc, v_meas
    global n_FL, n_FR, n_FL_prev, n_FR_prev
    global n_BL, n_BR, n_BL_prev, n_BR_prev

    n_FL = data.FL
    n_FR = data.FR
    n_BL = data.BL
    n_BR = data.BR

    # compute time elapsed
    tf = time.time()
    dt = tf - t0

    # if enough time elapse has elapsed, estimate v_x
    if dt >= dt_v_enc:
        # compute speed :  speed = distance / time
        v_FL = float(n_FL - n_FL_prev)*dx_qrt/dt
        v_FR = float(n_FR - n_FR_prev)*dx_qrt/dt
        v_BL = float(n_BL - n_BL_prev)*dx_qrt/dt
        v_BR = float(n_BR - n_BR_prev)*dx_qrt/dt

        # Uncomment/modify according to your encoder setup
        # v_meas    = (v_FL + v_FR)/2.0
        # Modification for 3 working encoders
        #v_meas = (v_FL + v_BL + v_BR)/3.0
        # Modification for bench testing (driven wheels only)
        #v_meas = sign*(v_FL + v_BL)/2.0
        v_meas = sign*v_FL

        # update old data
        n_FL_prev   = n_FL
        n_FR_prev   = n_FR
        n_BL_prev   = n_BL
        n_BR_prev   = n_BR
        t0          = time.time()


# state estimation node
def state_estimation():
    global dt_v_enc
    global v_meas, psi_meas
    # initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu/data', Imu, imu_callback)
    rospy.Subscriber('encoder', Encoder, enc_callback)
    rospy.Subscriber('ecu', ECU, ecu_callback)
    rospy.Subscriber('vel_sgn', vel_sgn, sign_callback)
    state_pub   = rospy.Publisher('state_estimate', Z_KinBkMdl, queue_size = 10)

    # get vehicle dimension parameters
    L_a = rospy.get_param("L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("L_b")       # distance from CoG to rear axel
    vhMdl   = (L_a, L_b)

    # get encoder parameters
    dt_v_enc = rospy.get_param("state_estimation/dt_v_enc") # time interval to compute v_x from encoders

    # get EKF observer properties
    q_std   = rospy.get_param("state_estimation/q_std")             # std of process noise
    r_std   = rospy.get_param("state_estimation/r_std")             # std of measurementnoise

    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    # estimation variables for Luemberger observer
    z_EKF       = zeros(4)

    # estimation variables for EKF
    P           = eye(4)                # initial dynamics coveriance matrix
    Q           = (q_std**2)*eye(4)     # process noise coveriance matrix
    R           = (r_std**2)*eye(2)     # measurement noise coveriance matrix

    while not rospy.is_shutdown():
        # publish state estimate
        (x, y, psi, v) = z_EKF

        # publish information
        state_pub.publish( Z_KinBkMdl(x, y, psi, v) )

        # collect measurements, inputs, system properties
        # collect inputs
        y   = array([psi_meas, v_meas])
        u   = array([ d_f, acc ])
        args = (u,vhMdl,dt)

        # apply EKF and get each state estimate
        (z_EKF,P) = ekf(f_KinBkMdl, z_EKF, P, h_KinBkMdl, y, Q, R, args )

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       state_estimation()
    except rospy.ROSInterruptException:
        pass
