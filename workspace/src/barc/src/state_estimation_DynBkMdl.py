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
from Localization_helpers import Localization
from barc.msg import ECU, Encoder, Z_DynBkMdl, pos_info
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from numpy import pi, cos, sin, eye, array, zeros, diag, arctan, tan, size, sign
from observers import kinematicLuembergerObserver, ekf
from system_models import f_KinBkMdl_predictive, h_KinBkMdl_predictive
from tf import transformations
from numpy import unwrap

# input variables
d_f         = 0
FxR         = 0

# raw measurement variables
yaw_prev = 0
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)

# from encoder
v_x_enc     = 0
t0          = time.time()
n_FL        = 0                     # counts in the front left tire
n_FR        = 0                     # counts in the front right tire
n_FL_prev   = 0
n_FR_prev   = 0
r_tire      = 0.04                  # radius from tire center to perimeter along magnets [m]
dx_qrt      = 2.0*pi*r_tire/4.0     # distance along quarter tire edge

x_meas      = 0
y_meas      = 0

# ecu command update
def ecu_callback(data):
    global FxR, d_f
    FxR         = data.motor        # input motor force [N]
    d_f         = data.servo        # input steering angle [rad]

# ultrasound gps data
def gps_callback(data):
    # units: [rad] and [rad/s]
    global x_meas, y_meas
    x_meas = data.x/100 # data is given in cm
    y_meas = data.y/100

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
        v_x_enc     = (v_FL + v_FR)/2.0#*cos(d_f)

        # update old data
        n_FL_prev   = n_FL
        n_FR_prev   = n_FR
        t0          = tf


# state estimation node
def state_estimation():
    # initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu/data', Imu, imu_callback)
    rospy.Subscriber('encoder', Encoder, enc_callback)
    rospy.Subscriber('ecu', ECU, ecu_callback)
    rospy.Subscriber('indoor_gps', Vector3, gps_callback)
    state_pub     = rospy.Publisher('state_estimate_dynamic', Z_DynBkMdl, queue_size = 1)     # size 1 -> when there's a newer message the older one is dropped
    state_pub_pos = rospy.Publisher('pos_info', pos_info, queue_size = 1)
    
    # get vehicle dimension parameters
    L_f = rospy.get_param("L_a")       # distance from CoG to front axel
    L_r = rospy.get_param("L_b")       # distance from CoG to rear axel
    m   = rospy.get_param("m")         # mass of vehicle
    I_z = rospy.get_param("I_z")       # moment of inertia about z-axis
    vhMdl   = (L_f, L_r)

    # get encoder parameters
    dt_vx   = rospy.get_param("state_estimation_dynamic/dt_v_enc")     # time interval to compute v_x

    # get tire model
    B   = rospy.get_param("tire_model/B")
    C   = rospy.get_param("tire_model/C")
    mu  = rospy.get_param("tire_model/mu")
    TrMdl = ([B,C,mu],[B,C,mu])

    # get external force model
    a0  = rospy.get_param("air_drag_coeff")
    Ff  = rospy.get_param("friction")

    # get EKF observer properties
    q_std       = rospy.get_param("state_estimation_dynamic/q_std")     # std of process noise
    psi_std     = rospy.get_param("state_estimation_dynamic/psi_std")   # std of measurementnoise
    v_std       = rospy.get_param("state_estimation_dynamic/v_std")
    gps_std     = rospy.get_param("state_estimation_dynamic/gps_std")   # std of gps measurements
    ang_v_std   = rospy.get_param("state_estimation_dynamic/ang_v_std") # std of gps measurements
    v_x_min     = rospy.get_param("state_estimation_dynamic/v_x_min")   # minimum velociy before using EKF
    est_mode    = rospy.get_param("state_estimation_dynamic/est_mode")  # estimation mode

    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    # estimation variables for Luemberger observer
    z_EKF       = zeros(8)

    # estimation variables for EKF
    P           = eye(8)                # initial dynamics coveriance matrix
    #Q           = (q_std**2)*eye(6)     # process noise coveriance matrixif est_mode==1:
    Q           = diag([0.1,0.1,0.1,0.1,0.01,0.01,0.01,0.01])    # values derived from inspecting P matrix during Kalman filter running

    if est_mode==1:                                     # use gps, IMU, and encoder
        R = diag([gps_std,gps_std,psi_std,v_std])**2
    elif est_mode==2:                                   # use IMU and encoder only
        R = diag([psi_std,v_std])**2
    elif est_mode==3:                                   # use gps only
        R = (gps_std**2)*eye(2)
    elif est_mode==4:                                   # use gps and angular velocity
        R = diag([gps_std,gps_std,psi_std,v_std])**2
    else:
        rospy.logerr("No estimation mode selected.")

    # Set up track parameters
    l = Localization()
    l.create_track()
    l.prepare_trajectory(0.06)


    w_z_f = 0         # filtered w_z (= angular velocity psiDot)
    while not rospy.is_shutdown():
        # publish state estimate
        (x,y,psi,v,x_pred,y_pred,psi_pred,v_pred) = z_EKF           # note, r = EKF estimate yaw rate

        # use Kalman values to predict state in 0.1s
        dt_pred = 0.0

        bta = arctan(L_f/(L_f+L_r)*tan(d_f))
        x_pred      = x   + dt_pred*( v*cos(psi + bta) )
        y_pred      = y   + dt_pred*( v*sin(psi + bta) ) 
        psi_pred    = psi + dt_pred*v/L_r*sin(bta)
        v_pred      = v   + dt_pred*(FxR - 0.63*sign(v)*v**2)
        v_x_pred    = cos(bta)*v_pred
        v_y_pred    = sin(bta)*v_pred
        w_z_f       = w_z_f + 0.2*(w_z-w_z_f)

        psi_dot_pred = w_z_f

        #state_pub.publish( Z_DynBkMdl(x,y,v_x,v_y,psi,psi_dot) )
        state_pub.publish( Z_DynBkMdl(x_pred,y_pred,v_x_pred,v_y_pred,psi_pred,psi_dot_pred) )

        # Update track position
        l.set_pos(x_pred,y_pred,psi_pred,v_x_pred,v_x_pred,v_y_pred,psi_dot_pred)        # v = v_x
        l.find_s()

        # and then publish position info
        state_pub_pos.publish( pos_info(l.s,l.ey,l.epsi,l.v,l.s_start,l.x,l.y,l.v_x,l.v_y,l.psi,l.psiDot,l.coeffX.tolist(),l.coeffY.tolist(),l.coeffTheta.tolist(),l.coeffCurvature.tolist()) )

        # apply EKF
        # get measurement
        y = array([x_meas,y_meas,yaw,v_x_enc])

        # define input
        u       = array([ d_f, FxR ])

        # build extra arguments for non-linear function
        args    = (u, vhMdl, dt) 

        # apply EKF and get each state estimate
        (z_EKF,P) = ekf(f_KinBkMdl_predictive, z_EKF, P, h_KinBkMdl_predictive, y, Q, R, args )
        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       state_estimation()
    except rospy.ROSInterruptException:
        pass
