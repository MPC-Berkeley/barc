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
from barc.msg import ECU, Vel_est, Z_KinBkMdl
from numpy import pi, cos, sin, eye, array, zeros, unwrap
from ekf import ekf
from system_models import f_KinBkMdl, h_KinBkMdl
from tf import transformations
from numpy import unwrap

# input variables [default values]
d_f         = 0         # steering angle [deg]
acc         = 0         # acceleration [m/s]

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

# ecu command update
def ecu_callback(data):
    global acc, d_f
    acc         = data.motor        # input acceleration
    d_f         = data.servo        # input steering angle

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
    global yaw_prev, yaw0, read_yaw0, yaw_local, psi_meas

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
    global v_meas
    v_meas = data.vel_est


# state estimation node
def state_estimation():
    global v_meas, psi_meas
    # initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu/data', Imu, imu_callback)
    rospy.Subscriber('vel_est', Vel_est, enc_callback)
    rospy.Subscriber('ecu', ECU, ecu_callback)
    state_pub   = rospy.Publisher('state_estimate', Z_KinBkMdl, queue_size = 10)

    # get vehicle dimension parameters
    L_a = rospy.get_param("L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("L_b")       # distance from CoG to rear axel
    vhMdl   = (L_a, L_b)

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
