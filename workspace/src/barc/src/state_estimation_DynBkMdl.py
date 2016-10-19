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
from barc.msg import ECU, Encoder, Z_DynBkMdl, pos_info, Vel_est
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from marvelmind_nav.msg import hedge_pos
from std_msgs.msg import Float32
from numpy import pi, cos, sin, eye, array, zeros, diag, arctan, tan, size, sign
from observers import kinematicLuembergerObserver, ekf
from system_models import f_KinBkMdl, h_KinBkMdl, f_KinBkMdl_psi_drift, h_KinBkMdl_psi_drift
from tf import transformations
from numpy import unwrap
from numpy import *

# input variables
cmd_servo   = 0
cmd_motor   = 0
d_f         = 0
FxR         = 0

# raw measurement variables
yaw_prev = 0
yaw0     = 0            # yaw at t = 0
yaw      = 0
(roll_meas, pitch_meas, yaw_meas, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)
imu_times = [0]*25
psiDot_hist = [0]*25

# Velocity
vel_est         = 0
vel_est_hist    = [0]*2
vel_times       = [0]*2

# GPS
x_meas      = 0
y_meas      = 0

x_hist      = [0]*30
y_hist      = [0]*30
gps_times   = [0]*30

poly_x      = [0]*3              # polynomial coefficients for x and y position measurement (2nd order)
poly_y      = [0]*3

t0          = 0
running     = False

# ecu command update
def ecu_callback(data):
    global FxR, cmd_motor, cmd_servo, running
    cmd_motor         = data.motor        # input motor force [N]
    cmd_servo         = data.servo        # input steering angle [rad]
    FxR               = cmd_motor
    if not running:                 # set 'running' to True once the first command is received -> here yaw is going to be set to zero
        running = True

# ultrasound gps data
def gps_callback(data):
    # units: [rad] and [rad/s]
    global x_meas, y_meas, t0
    x_meas_pred = polyval(poly_x,data.timestamp_ros-t0)    # predict new position
    y_meas_pred = polyval(poly_y,data.timestamp_ros-t0)

    if abs(x_meas_pred-data.x_m) < 0.5 and abs(y_meas_pred-data.y_m) < 0.5 or not running:  # check for outlier
        x_meas = data.x_m # data is given in cm
        y_meas = data.y_m

        x_hist.append(x_meas)
        y_hist.append(y_meas)
        gps_times.append(data.timestamp_ros-t0)
        x_hist.pop(0)
        y_hist.pop(0)
        gps_times.pop(0)

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global roll_meas, pitch_meas, yaw_meas, a_x, a_y, a_z, w_x, w_y, w_z
    global yaw_prev, yaw0, yaw
    global imu_times, psiDot_hist

    # get orientation from quaternion data, and convert to roll, pitch, yaw
    # extract angular velocity and linear acceleration data
    ori  = data.orientation
    quaternion  = (ori.x, ori.y, ori.z, ori.w)
    (roll_meas, pitch_meas, yaw_meas) = transformations.euler_from_quaternion(quaternion)
    # yaw_meas is element of [-pi,pi]
    yaw = unwrap([yaw_prev,yaw_meas])[1]            # get smooth yaw (from beginning)
    yaw_prev = yaw                                  # and always use raw measured yaw for unwrapping
    # from this point on 'yaw' will be definitely unwrapped (smooth)!
    if not running:
        yaw0 = yaw              # set yaw0 to current yaw
        yaw = 0                 # and current yaw to zero
    else:
        yaw = yaw - yaw0

    imu_times.append(data.header.stamp.to_sec()-t0)
    imu_times.pop(0)
    
    # extract angular velocity and linear acceleration data
    w_x = data.angular_velocity.x
    w_y = data.angular_velocity.y
    w_z = data.angular_velocity.z
    a_x = data.linear_acceleration.x
    a_y = data.linear_acceleration.y
    a_z = data.linear_acceleration.z

    psiDot_hist.append(w_z)
    psiDot_hist.pop(0)

def vel_est_callback(data):
    global vel_est, t0, vel_est_hist, vel_times
    if not data.vel_est == vel_est or not running:        # if we're receiving a new measurement
        vel_est = data.vel_est
        vel_est_hist.append(vel_est)
        vel_est_hist.pop(0)
        vel_times.append(data.stamp.to_sec()-t0)
        vel_times.pop(0)

# state estimation node
def state_estimation():
    global t0, poly_x, poly_y
    # initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu/data', Imu, imu_callback)
    rospy.Subscriber('vel_est', Vel_est, vel_est_callback)
    rospy.Subscriber('ecu', ECU, ecu_callback)
    rospy.Subscriber('hedge_pos', hedge_pos, gps_callback)
    state_pub_pos = rospy.Publisher('pos_info', pos_info, queue_size = 1)
    
    # get vehicle dimension parameters
    L_f = rospy.get_param("L_a")       # distance from CoG to front axel
    L_r = rospy.get_param("L_b")       # distance from CoG to rear axel
    vhMdl   = (L_f, L_r)

    # get EKF observer properties
    q_std       = rospy.get_param("state_estimation_dynamic/q_std")     # std of process noise
    psi_std     = rospy.get_param("state_estimation_dynamic/psi_std")   # std of measurementnoise
    v_std       = rospy.get_param("state_estimation_dynamic/v_std")     # std of velocity estimation
    gps_std     = rospy.get_param("state_estimation_dynamic/gps_std")   # std of gps measurements
    ang_v_std   = rospy.get_param("state_estimation_dynamic/ang_v_std") # std of angular velocity measurements
    est_mode    = rospy.get_param("state_estimation_dynamic/est_mode")  # estimation mode

    # set node rate
    loop_rate   = 25
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = rospy.get_rostime().to_sec()

    # settings about psi estimation (use different models accordingly)
    psi_drift_active = True
    psi_drift = 0

    if psi_drift_active:
        z_EKF       = zeros(5)
        P           = eye(5)                # initial dynamics coveriance matrix
        Q           = diag([5.0,5.0,5.0,10.0,1.0])*dt
    else:
        z_EKF       = zeros(4)
        P           = eye(4)                # initial dynamics coveriance matrix
        Q           = diag([5.0,5.0,5.0,5.0])*dt

    if est_mode==1:                                     # use gps, IMU, and encoder
        print("Using GPS, IMU and encoders.")
        R = diag([gps_std,gps_std,psi_std,v_std])**2
    elif est_mode==2:                                   # use IMU and encoder only
        print("Using IMU and encoders.")
        R = diag([psi_std,v_std])**2
    elif est_mode==3:                                   # use gps only
        print("Using GPS.")
        R = (gps_std**2)*eye(2)
    elif est_mode==4:                                   # use gps and encoder
        print("Using GPS and encoders.")
        R = diag([gps_std,gps_std,v_std])**2
    else:
        rospy.logerr("No estimation mode selected.")

    # Set up track parameters
    l = Localization()
    l.create_track()
    l.prepare_trajectory(0.06)

    psi_prev = 0        # for debugging

    d_f = 0

    while not rospy.is_shutdown():
        t = rospy.get_rostime().to_sec()-t0           # current time

        # calculate new steering angle (low pass filter on steering input)
        d_f = d_f + (cmd_servo-d_f)*0.25

        # update x and y polynomial:
        t_matrix            = vstack((gps_times,gps_times,ones(size(gps_times))))
        t_matrix[0]         = t_matrix[0]**2
        poly_x              = linalg.lstsq(t_matrix.T,x_hist)[0]
        poly_y              = linalg.lstsq(t_matrix.T,y_hist)[0]
        # calculate current position from interpolated measurements
        x_meas_pred         = polyval(poly_x,t)
        y_meas_pred         = polyval(poly_y,t)

        # print "Times:"
        # print gps_times
        # print "x-values:"
        # print x_hist
        # print "ROS time: %f"%t
        # print "Polynomial coefficients:"
        # print poly_x
        # print "tdif = %f"%(t-gps_times[-1])
        # print "Pred. x_meas: %f, real x_meas: %f"%(x_meas_pred,x_meas)

        # update velocity estimation polynomial:
        t_matrix_vel        = vstack((vel_times,ones(size(vel_times))))
        poly_vel            = linalg.lstsq(t_matrix_vel.T,vel_est_hist)[0]
        vel_est_pred        = polyval(poly_vel,t)
        # print "Times:"
        # print vel_times
        # print "Meas:"
        # print vel_est_hist
        # print "ROS time: %f"%t
        # print "tdif = %f"%(t-vel_times[-1])
        # print "Pred. vel: %f, real vel_est: %f"%(vel_est_pred,vel_est)

        # update IMU polynomial:
        t_matrix_imu        = vstack((imu_times,imu_times,ones(size(imu_times))))
        t_matrix_imu[0]     = t_matrix_imu[0]**2
        poly_psiDot         = linalg.lstsq(t_matrix_imu.T,psiDot_hist)[0]
        psiDot_meas_pred    = polyval(poly_psiDot,t)

        if psi_drift_active:
            (x,y,psi,v,psi_drift) = z_EKF           # note, r = EKF estimate yaw rate
        else:
            (x,y,psi,v) = z_EKF           # note, r = EKF estimate yaw rate

        # use Kalman values to predict state in 0.1s
        dt_pred = 0.0

        bta = arctan(L_f/(L_f+L_r)*tan(d_f))
        x_pred      = x   + dt_pred*( v*cos(psi + bta) )
        y_pred      = y   + dt_pred*( v*sin(psi + bta) ) 
        psi_pred    = psi + dt_pred*v/L_r*sin(bta)
        v_pred      = v   + dt_pred*(FxR - 0.63*sign(v)*v**2)
        v_x_pred    = cos(bta)*v_pred
        v_y_pred    = sin(bta)*v_pred

        psi_dot_pred = psiDot_meas_pred

        # Update track position
        l.set_pos(x_pred,y_pred,psi_pred,v_x_pred,v_x_pred,v_y_pred,psi_dot_pred)        # v = v_x
        l.find_s()

        # and then publish position info
        state_pub_pos.publish( pos_info(l.s,l.ey,l.epsi,l.v,l.s_start,l.x,l.y,l.v_x,l.v_y,l.psi,l.psiDot,l.coeffX.tolist(),l.coeffY.tolist(),l.coeffTheta.tolist(),l.coeffCurvature.tolist()) )

        # apply EKF
        # get measurement
        if est_mode==1:
            y = array([x_meas_pred,y_meas_pred,yaw,vel_est])
        elif est_mode==2:
            y = array([yaw,vel_est])
        elif est_mode==3:
            y = array([x_meas,y_meas])
        elif est_mode==4:
            y = array([x_meas,y_meas,vel_est])
        else:
            print("Wrong estimation mode specified.")

        # define input
        u       = array([ d_f, FxR ])

        # build extra arguments for non-linear function
        args    = (u, vhMdl, dt, est_mode)

        # apply EKF and get each state estimate
        if psi_drift_active:
            (z_EKF,P) = ekf(f_KinBkMdl_psi_drift, z_EKF, P, h_KinBkMdl_psi_drift, y, Q, R, args )
        else:
            (z_EKF,P) = ekf(f_KinBkMdl, z_EKF, P, h_KinBkMdl, y, Q, R, args )

        #print("yaw = %f, psi = %f"%(yaw,psi_pred))
        if abs(psi_pred-psi_prev) > 0.2:
            print("WAAAAAAAAAAAAARNING LARGE PSI DIFFERENCE!!!!!!!!!!!!!!!!!!!******************\n")
            print("*****************************************************************************\n")
        psi_prev = psi_pred
        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       state_estimation()
    except rospy.ROSInterruptException:
        pass
