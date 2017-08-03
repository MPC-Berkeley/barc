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
from barc.msg import ECU, Encoder, Z_KinBkMdl
from marvelmind_nav.msg import hedge_pos
from numpy import pi, cos, sin, eye, array, zeros, unwrap, diag
from numpy import vstack, ones, polyval, linalg, append
from ekf_CT import ekf
from system_models_CT import f_KinBkMdl, h_KinBkMdl_withGPS
from tf import transformations
from numpy import unwrap
import scipy.io as sio
import numpy as np

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
t0_enc      = time.time()
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

# gps
x_hist = zeros(15)
y_hist = zeros(15)
t_gps = zeros(15)
c_X = array([0,0,0])
c_Y = array([0,0,0])
read_gps0 = False

# Estimator data
x_est = 0.0
y_est = 0.0

message_kin = {}
index_imu = 0
index_est = 0
index_ecu = 0
index_enc = 0
index_gps = 0
message_ecu = {}
message_imu = {}
message_enc = {}
message_est = {}
message_gps = {}

x0 = 0
y0 = 0

# ecu command update
def ecu_callback(data):
    global acc, d_f, t0
    global message_ecu, index_ecu
    t_ecu = time.time() - t0

    index_ecu += 1
    
    acc         = data.motor        # input acceleration
    d_f         = data.servo        # input steering angle
    message_ecu[('index'+str(index_ecu))] = np.array([t_ecu, acc, d_f])

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
    global yaw_prev, yaw0, read_yaw0, yaw_local, psi_meas
    global message_imu, index_imu, t0
    t_imu = time.time() - t0

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
    index_imu += 1
    message_imu[('index'+str(index_imu))] = np.array([t_imu, roll, pitch, yaw, w_x, w_y, w_z, a_x, a_y, a_z])

# encoder measurement update
def enc_callback(data):
    global v, t0, dt_v_enc, v_meas, t0_enc
    global n_FL, n_FR, n_FL_prev, n_FR_prev
    global n_BL, n_BR, n_BL_prev, n_BR_prev
    global message_enc, index_enc
    t_enc = time.time() - t0

    n_FL = data.FL
    n_FR = data.FR
    n_BL = data.BL
    n_BR = data.BR

    # compute time elapsed
    tf = time.time()
    dt = tf - t0_enc

    # if enough time elapse has elapsed, estimate v_x
    if dt >= dt_v_enc:
        # compute speed :  speed = distance / time
        v_FL = float(n_FL - n_FL_prev)*dx_qrt/dt
        v_FR = float(n_FR - n_FR_prev)*dx_qrt/dt
        v_BL = float(n_BL - n_BL_prev)*dx_qrt/dt
        v_BR = float(n_BR - n_BR_prev)*dx_qrt/dt

        # Uncomment/modify according to your encoder setup
        # v_meas    = (v_FL + v_FR)/2.0dt_v_enc
        # Modification for 3 working encoders
        v_meas = (v_FL + v_FR + v_BL)/3.0
        # Modification for bench testing (driven wheels only)
        # v = (v_BL + v_BR)/2.0

        # update old data
        n_FL_prev   = n_FL
        n_FR_prev   = n_FR
        n_BL_prev   = n_BL
        n_BR_prev   = n_BR
        t0_enc          = time.time()

        index_enc += 1
        message_enc[('index'+str(index_enc))] = np.array([t_enc, v_FL, v_FR, v_BL])

# GPS measurement update
#def gps_callback(data):
#	global x_meas, y_meas, x0, y0
#	global read_gps0

#	x_meas = data.x_m;
#	y_meas = data.y_m;

#	if not read_gps0:
#		read_gps0 = True
#       x0 = x_meas
#        y0 = y_meas

def gps_callback(data):
    """This function is called when a new GPS signal is received."""
    global x_est, y_est
    global x_hist, y_hist, t_gps, t0
    global c_X, c_Y, read_gps0, x0, y0
    global index_gps, message_gps

    x_meas = data.x_m
    y_meas = data.y_m

    t_now = time.time()-t0
    if not read_gps0:
        read_gps0 = True
        x0 = x_meas
        y0 = y_meas
    
    x_meas = data.x_m - x0
    y_meas = data.y_m - y0

    dist = (x_est-data.x_m)**2 + (y_est-data.y_m)**2

    if dist < 1.0:
        x_hist = append(x_hist, data.x_m)
        y_hist = append(y_hist, data.y_m)
        t_gps = append(t_gps, t_now)

    x_hist = x_hist[t_gps > t_now-1.0]
    y_hist = y_hist[t_gps > t_now-1.0]
    t_gps = t_gps[t_gps > t_now-1.0]

    sz = np.shape(t_gps)[0]
    if sz > 4:
        t_matrix = vstack([t_gps**2, t_gps, ones(sz)]).T
        c_X = linalg.lstsq(t_matrix, x_hist)[0]
        c_Y = linalg.lstsq(t_matrix, y_hist)[0]

    index_gps += 1
    message_gps['index'+str(index_gps)] = np.array([t_now, data.x_m, data.y_m])

# state estimation node
def state_estimation():
    global dt_v_enc,t0
    global v_meas, psi_meas
    global message_est, index_est
    global c_X, c_Y

    # initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu/data', Imu, imu_callback)
    rospy.Subscriber('encoder', Encoder, enc_callback)
    rospy.Subscriber('ecu', ECU, ecu_callback)
    rospy.Subscriber('hedge_pos', hedge_pos , gps_callback)
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
    #t0          = time.time()

    # estimation variables for Luemberger observer
    z_EKF       = array([0 ,0 ,0.,0.])

    # estimation variables for EKF
    var_gps = 4.0e-05
    P           = eye(4)                # initial dynamics coveriance matrix
    Q           = (q_std**2)*eye(4)     # process noise coveriance matrix
    R           = diag(array([var_gps, var_gps, (r_std**2), (r_std**2)]))     # measurement noise coveriance matrix

    while not rospy.is_shutdown():

        # publish state estimate
        (x, y, psi, v) = z_EKF

        # publish information
        state_pub.publish( Z_KinBkMdl(x, y, psi, v) )

        # collect measurements, inputs, system properties
        # collect inputs
        t_est = time.time() - t0
        x_meas = polyval(c_X, t_est)
        y_meas = polyval(c_Y, t_est)
        y_ekf   = array([x_meas,y_meas,psi_meas, v_meas])
        u_ekf   = array([ d_f, acc ])
        args = (u_ekf,vhMdl,dt)

        # apply EKF and get each state estimate
        # (z_EKF,P) = ekf(f_KinBkMdl, z_EKF, P, h_KinBkMdl, y, Q, R, args )
        (z_EKF,P) = ekf(f_KinBkMdl, z_EKF, P, h_KinBkMdl_withGPS, y_ekf, Q, R, args )

        x_est = z_EKF[0]
        y_est = z_EKF[1]

        index_est += 1
        # message_est[index_est] = np.array([psi_meas, v_meas, d_f, acc, x, y, psi, v])
        message_est[('index'+str(index_est))] = np.array([t_est, psi_meas, v_meas, d_f, acc, x, y, psi, v, x_meas, y_meas])
        # print(message_est)
        if (index_est>0) and (index_est%50 == 0):
            message_kin["est"]=message_est
            message_kin["enc"]=message_enc
            message_kin["imu"]=message_imu
            message_kin["ecu"]=message_ecu
            message_kin["gps"]=message_gps
            

            sio.savemat('./message_kin_a3df35.mat', {'message_kin':message_kin})
        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       state_estimation()
    except rospy.ROSInterruptException:
        pass
