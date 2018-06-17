#!/usr/bin/env python
"""
    File name: state_estimator.py
    Author: Shuqi Xu
    Email: shuqixu@kth.se
    Python Version: 2.7.12
"""
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
import sys
sys.path.append(sys.path[0]+'/ControllersObject')
sys.path.append(sys.path[0]+'/Utilities')
import rospy
from barc.msg import ECU, pos_info, Vel_est
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_imu_fusion, hedge_pos
from std_msgs.msg import Header
from numpy import eye, zeros, diag, tan, cos, sin, vstack, linalg, pi
from numpy import ones, polyval, size, dot, add
from scipy.linalg import inv, cholesky
from tf import transformations
import math
import numpy as np
import os
import pdb

def main():
    # node initialization
    rospy.init_node("state_estimation")
    a_delay     = rospy.get_param("state_estimator_2/delay_a")
    df_delay    = rospy.get_param("state_estimator_2/delay_df")
    loop_rate   = 50.0
   

    Q = eye(8)
    Q[0,0]  =  0.01     # x
    Q[1,1]  =  0.01     # y
    Q[2,2]  =  0.01     # vx
    Q[3,3]  =  0.01     # vy
    Q[4,4]  =  1.0      # ax
    Q[5,5]  =  1.0      # ay 
    Q[6,6]  = 10.0      # psi
    Q[7,7]  = 10.0      # psiDot
    R = eye(7)
    R[0,0]  = 20.0      # x
    R[1,1]  = 20.0      # y
    R[2,2]  =  0.1      # vx
    R[3,3]  = 10.0      # ax 
    R[4,4]  = 10.0      # ay 
    R[5,5]  =  0.1      # psiDot
    R[6,6]  =  0.01    # vy
    thReset =  1.5

    # Q_noVy = eye(8)
    # Q_noVy[0,0] =  0.01  # x
    # Q_noVy[1,1] =  0.01  # y
    # Q_noVy[2,2] =  0.01  # vx
    # Q_noVy[3,3] =  0.01  # vy
    # Q_noVy[4,4] =   1.0  # ax
    # Q_noVy[5,5] =   1.0  # ay
    # Q_noVy[6,6] = 10.0   # psi
    # Q_noVy[7,7] = 10.0   # psidot
    # # Q[8,8] = 0.0 # psiDot in the model
    # R_noVy = eye(6)
    # R_noVy[0,0]  = 20.0   # x
    # R_noVy[1,1]  = 20.0   # y
    # R_noVy[2,2]  = 0.1   # vx
    # R_noVy[3,3]  = 10.0   # ax
    # R_noVy[4,4]  = 30.0   # ay
    # R_noVy[5,5]  = 0.10    # psiDot
    # R[6,6]       = 0.001    # vy
    # thReset_noVy = 0.8

    t0 = rospy.get_rostime().to_sec()
    imu = ImuClass(t0)
    gps = GpsClass(t0)
    enc = EncClass(t0)
    ecu = EcuClass(t0)

    est     = Estimator(t0,loop_rate,a_delay,df_delay,Q,  R,   thReset)

    estMsg = pos_info()
    
    saved_x_est      = []
    saved_y_est      = []
    saved_vx_est     = []
    saved_vy_est     = []
    saved_psi_est    = []
    saved_psiDot_est = []
    saved_ax_est     = []
    saved_ay_est     = []
    saved_switch     = []

    while not rospy.is_shutdown():

        if (est.vx_est + 0.0 * np.abs(est.psiDot_est) ) > 0.8 or (np.abs(est.psiDot_est) > 1.0):
            flagVy      = True
            # print "================ Not using vy! =============="
        else:
            flagVy      = False
            # print "================ Using vy! =============="

        est.estimateState(imu,gps,enc,ecu,est.ekf,flagVy)
        
        estMsg.header.stamp = rospy.get_rostime()

        estMsg.v        = np.sqrt(est.vx_est**2 + est.vy_est**2)
        estMsg.x        = est.x_est 
        estMsg.y        = est.y_est
        estMsg.v_x      = est.vx_est 
        estMsg.v_y      = est.vy_est
        estMsg.psi      = est.yaw_est
        estMsg.psiDot   = est.psiDot_est
        estMsg.a_x      = est.ax_est
        estMsg.a_y      = est.ay_est
        estMsg.u_a      = ecu.a
        estMsg.u_df     = ecu.df

        est.state_pub_pos.publish(estMsg)

        # Save estimator output.
        # NEED TO DO IT HERE AS THERE ARE MULTIPLE ESTIMATOR RUNNING IN PARALLEL
        saved_x_est.append(estMsg.x)
        saved_y_est.append(estMsg.y)
        saved_vx_est.append(estMsg.v_x)
        saved_vy_est.append(estMsg.v_y)
        saved_psi_est.append(estMsg.psi)
        saved_psiDot_est.append(estMsg.psiDot)
        saved_ax_est.append(estMsg.a_x)
        saved_ay_est.append(estMsg.a_y)
        saved_switch.append(int(flagVy))

        est.rate.sleep()

    homedir = os.path.expanduser("~")
    pathSave = os.path.join(homedir,"barc_data/estimator_output.npz")
    np.savez(pathSave,yaw_est_his       = saved_psi_est,
                      psiDot_est_his    = saved_psiDot_est,
                      x_est_his         = saved_x_est,
                      y_est_his         = saved_y_est,
                      vx_est_his        = saved_vx_est,
                      vy_est_his        = saved_vy_est,
                      ax_est_his        = saved_ax_est,
                      ay_est_his        = saved_ay_est,
                      gps_time          = est.gps_time,
                      imu_time          = est.imu_time,
                      enc_time          = est.enc_time,
                      inp_x_his         = est.x_his,
                      inp_y_his         = est.y_his,
                      inp_v_meas_his    = est.v_meas_his,
                      inp_ax_his        = est.ax_his,
                      inp_ay_his        = est.ay_his,
                      inp_psiDot_his    = est.psiDot_his,
                      inp_a_his         = est.inp_a_his,
                      inp_df_his        = est.inp_df_his,
                      flagVy            = saved_switch)

    pathSave = os.path.join(homedir,"barc_data/estimator_imu.npz")
    np.savez(pathSave,psiDot_his    = imu.psiDot_his,
                      roll_his      = imu.roll_his,
                      pitch_his     = imu.pitch_his,
                      yaw_his       = imu.yaw_his,
                      ax_his        = imu.ax_his,
                      ay_his        = imu.ay_his,
                      imu_time      = imu.time_his)

    pathSave = os.path.join(homedir,"barc_data/estimator_gps.npz")
    np.savez(pathSave,x_his         = gps.x_his,
                      y_his         = gps.y_his,
                      gps_time      = gps.time_his)

    pathSave = os.path.join(homedir,"barc_data/estimator_enc.npz")
    np.savez(pathSave,v_fl_his          = enc.v_fl_his,
                      v_fr_his          = enc.v_fr_his,
                      v_rl_his          = enc.v_rl_his,
                      v_rr_his          = enc.v_rr_his,
                      enc_time          = enc.time_his)

    pathSave = os.path.join(homedir,"barc_data/estimator_ecu.npz")
    np.savez(pathSave,a_his         = ecu.a_his,
                      df_his        = ecu.df_his,
                      ecu_time      = ecu.time_his)

    print "Finishing saveing state estimation data"

class Estimator(object):
    """ Object collecting  estimated state data
    Attributes:
        Estimated states:
            1.x_est     2.y_est
            3.vx_est    4.vy_est        5.v_est
            6.ax_est    7.ay_est
            8.yaw_est   9.psiDot_est    10.psiDrift_est
        Estimated states history:
            1.x_est_his     2.y_est_his
            3.vx_est_his    4.vy_est_his        5.v_est_his
            6.ax_est_his    7.ay_est_his
            8.yaw_est_his   9.psiDot_est_his    10.psiDrift_est_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    Methods:
        stateEstimate(imu,gps,enc,ecu):
            Estimate current state from sensor data
        ekf(y,u):
            Extended Kalman filter
        ukf(y,u):
            Unscented Kalman filter
        numerical_jac(func,x,u):
            Calculate jacobian numerically
        f(x,u):
            System prediction model
        h(x,u):
            System measurement model
    """

    def __init__(self,t0,loop_rate,a_delay,df_delay,Q,R,thReset):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        self.thReset = thReset

        dt          = 1.0 / loop_rate
        self.rate   = rospy.Rate(loop_rate)
        L_f         = rospy.get_param("L_a")       # distance from CoG to front axel
        L_r         = rospy.get_param("L_b")       # distance from CoG to rear axel
        self.vhMdl  = (L_f, L_r)
        self.Q      = Q
        self.R      = R
        self.P      = np.eye(np.size(Q,0)) # initializationtial covariance matrix
        self.z      = np.zeros(np.size(Q,0)) # initial state mean
        self.dt     = dt
        self.a_delay        = a_delay
        self.df_delay       = df_delay
        self.a_his          = [0.0]*int(a_delay/dt)
        self.df_his         = [0.0]*int(df_delay/dt)

        self.state_pub_pos  = rospy.Publisher('pos_info', pos_info, queue_size=1)
        self.t0             = t0

        self.x_est          = 0.0
        self.y_est          = 0.0
        self.vx_est         = 0.0
        self.vy_est         = 0.0
        self.v_est          = 0.0
        self.ax_est         = 0.0
        self.ay_est         = 0.0
        self.yaw_est        = 0.0
        self.psiDot_est     = 0.0
        self.psiDrift_est   = 0.0
        self.curr_time      = rospy.get_rostime().to_sec()

        self.x_est_his          = []
        self.y_est_his          = []
        self.vx_est_his         = []
        self.vy_est_his         = []
        self.v_est_his          = []
        self.ax_est_his         = []
        self.ay_est_his         = []
        self.yaw_est_his        = []
        self.psiDot_est_his     = []
        self.time_his           = []

        # SAVE THE measurement/input SEQUENCE USED BY KF
        self.x_his      = []
        self.y_his      = []
        self.v_meas_his = []
        self.ax_his     = []
        self.ay_his     = []
        self.psiDot_his = []
        self.inp_a_his  = []
        self.inp_df_his = []

        self.gps_time = []
        self.enc_time = []
        self.imu_time = []

        self.oldGPS_x = 0.0
        self.oldGPS_y = 0.0

    # ecu command update
    def estimateState(self, imu, gps, enc, ecu, KF, flagVy):
        """Do extended Kalman filter to estimate states"""
        self.curr_time = rospy.get_rostime().to_sec()

        self.a_his.append(ecu.a)
        self.df_his.append(ecu.df)
        u = [self.a_his.pop(0), self.df_his.pop(0)]
        # u = [ecu.a, self.df_his.pop(0)]
        
        bta = 0.5 * u[1]
        dist   = np.sqrt(( self.x_est - gps.x )**2 + ( self.y_est - gps.y )**2)

        if flagVy == False:
            y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot, bta * enc.v_meas])
        else:
            y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot])

        if np.abs(imu.psiDot) < self.thReset:
            self.z[3] = 0.0

        KF(y,u, flagVy)


        # SAVE THE measurement/input SEQUENCE USED BY KF
        self.x_his.append(gps.x)
        self.y_his.append(gps.y)
        self.v_meas_his.append(enc.v_meas)
        self.ax_his.append(imu.ax)
        self.ay_his.append(imu.ay)
        self.psiDot_his.append(imu.psiDot)
        self.inp_a_his.append(u[0])
        self.inp_df_his.append(u[1])

        self.gps_time.append(gps.curr_time)
        self.imu_time.append(imu.curr_time)
        self.enc_time.append(enc.curr_time)
        # SAVE output KF given the above measurements
        self.saveHistory()

    def ekf(self, y, u, flagVy):
        """
        EKF   Extended Kalman Filter for nonlinear dynamic systems
        ekf(f,mx,P,h,z,Q,R) returns state estimate, x and state covariance, P 
        for nonlinear dynamic system:
                  x_k+1 = f(x_k) + w_k
                  y_k   = h(x_k) + v_k
        where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
              v ~ N(0,R) meaning v is gaussian noise with covariance R
        Inputs:    f: function handle for f(x)
                   z_EKF: "a priori" state estimate
                   P: "a priori" estimated state covariance
                   h: fanction handle for h(x)
                   y: current measurement
                   Q: process noise covariance 
                   R: measurement noise covariance
                   args: additional arguments to f(x, *args)
        Output:    mx_kp1: "a posteriori" state estimate
                   P_kp1: "a posteriori" state covariance
                   
        Notation: mx_k = E[x_k] and my_k = E[y_k], where m stands for "mean of"
        """
        numericalDiffActive = True
        
        xDim    = self.z.size                           # dimension of the state

        mx_kp1, Aa  = self.f(self.z, u)                               # predict next state
        An          = self.numerical_jac(self.f, self.z, u, flagVy)  # linearize process model about current state

        if numericalDiffActive == True:
            A = An
        else:
            A = Aa

        P_kp1   = dot(dot(A,self.P),A.T) + self.Q                 # proprogate variance

        my_kp1, Ha   = self.h(mx_kp1, u, flagVy)                      # predict future output
        Hn           = self.numerical_jac(self.h, mx_kp1, u, flagVy)  # linearize measurement model about predicted next state

        if numericalDiffActive == True:    
            H = Hn
        else:
            H = Ha

        P12     = dot(P_kp1, H.T)                                 # cross covariance

        if flagVy == False:
            K       = dot(P12, inv( dot(H,P12) + self.R))       # Kalman filter gain
        else:
            K       = dot(P12, inv( dot(H,P12) + self.R[:-1,:-1]))       # Kalman filter gain
            
        self.z  = mx_kp1 + dot(K,(y - my_kp1))

        if flagVy == False:
            self.P  = dot(dot(K,self.R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )
        else:
            self.P  = dot(dot(K,self.R[:-1,:-1]),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )

        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z

    def numerical_jac(self,func,x,u, flagVy):
        """
        Function to compute the numerical jacobian of a vector valued function 
        using final differences
        """
        # numerical gradient and diagonal hessian
        y, _ = func(x,u, flagVy)
        
        jac = zeros( (y.size,x.size) )
        eps = 1e-5
        xp = np.copy(x)
        
        for i in range(x.size):
            xp[i] = x[i] + eps/2.0
            yhi, _ = func(xp, u, flagVy)
            xp[i] = x[i] - eps/2.0
            ylo, _ = func(xp, u, flagVy)
            xp[i] = x[i]
            jac[:,i] = (yhi - ylo) / eps
        return jac

    def f(self, z, u, flagVy=True):
        """ This Sensor model contains a pure Sensor-Model and a Kinematic model. They're independent from each other."""
        dt = self.dt
        zNext = [0]*8
        zNext[0] = z[0] + dt*(cos(z[6])*z[2] - sin(z[6])*z[3])  # x
        zNext[1] = z[1] + dt*(sin(z[6])*z[2] + cos(z[6])*z[3])  # y
        zNext[2] = z[2] + dt*(z[4]+z[7]*z[3])                   # v_x
        zNext[3] = z[3] + dt*(z[5]-z[7]*z[2])                   # v_y
        zNext[4] = z[4]                                         # a_x
        zNext[5] = z[5]                                         # a_y
        zNext[6] = z[6] + dt*(z[7])                             # psi
        zNext[7] = z[7]                                         # psidot

        jac      = np.array([[1.0,          0.0,  dt*cos(z[6]), -dt*sin(z[6]), 0.0, 0.0, dt*(-sin(z[6])*z[2]-cos(z[6])*z[3]),     0.0],
                             [0.0,          1.0,  dt*sin(z[6]),  dt*cos(z[6]), 0.0, 0.0, dt*( cos(z[6])*z[2]-sin(z[6])*z[3]),     0.0],
                             [0.0,          0.0,           1.0,       dt*z[7],  dt, 0.0,                                 0.0, dt*z[3]],
                             [0.0,          0.0,      -dt*z[7],           1.0, 0.0,  dt,                                 0.0,-dt*z[2]],
                             [0.0,          0.0,           0.0,           0.0, 1.0, 0.0,                                 0.0,     0.0],
                             [0.0,          0.0,           0.0,           0.0, 0.0, 1.0,                                 0.0,     0.0],
                             [0.0,          0.0,           0.0,           0.0, 0.0, 0.0,                                 1.0,      dt],
                             [0.0,          0.0,           0.0,           0.0, 0.0, 0.0,                                 0.0,     1.0]])

        return np.array(zNext), jac

    def h(self, x, u, flagVy):
        """ This is the measurement model to the kinematic<->sensor model above """
        if flagVy == False:
            y = [0]*7
            y[0] = x[0]   # x
            y[1] = x[1]   # y
            y[2] = x[2]   # vx
            y[3] = x[4]   # a_x
            y[4] = x[5]   # a_y
            y[5] = x[7]   # psiDot
            y[6] = x[3]   # vy

            jac = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,],
                            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,],
                            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,],
                            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,],
                            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,]])
        else:
            y = [0]*6
            y[0] = x[0]   # x
            y[1] = x[1]   # y
            y[2] = x[2]   # vx
            y[3] = x[4]   # a_x
            y[4] = x[5]   # a_y
            y[5] = x[7]   # psiDot

            jac = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
        
        return np.array(y), jac

    def saveHistory(self):
        self.time_his.append(self.curr_time)

        self.x_est_his.append(self.x_est)
        self.y_est_his.append(self.y_est)
        self.vx_est_his.append(self.vx_est)
        self.vy_est_his.append(self.vy_est)
        self.v_est_his.append(self.v_est)
        self.ax_est_his.append(self.ax_est)
        self.ay_est_his.append(self.ay_est)
        self.yaw_est_his.append(self.yaw_est)
        self.psiDot_est_his.append(self.psiDot_est)


# ========================================================================================================================================
# ======================================================= SENSOR CLASSES =================================================================
# ========================================================================================================================================

class ImuClass(object):
    """ Object collecting GPS measurement data
    Attributes:
        Measurement:
            1.yaw 2.psiDot 3.ax 4.ay 5.roll 6.pitch
        Measurement history:
            1.yaw_his 2.psiDot_his 3.ax_his 4.ay_his 5.roll_his 6.pitch_his
        Time stamp
            1.t0 2.time_his
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """

        rospy.Subscriber('imu/data', Imu, self.imu_callback, queue_size=1)

        # Imu measurement
        self.yaw     = 0.0
        self.psiDot  = 0.0
        self.ax      = 0.0
        self.ay      = 0.0
        self.roll    = 0.0
        self.pitch   = 0.0
        
        # Imu measurement history
        self.yaw_his     = []
        self.psiDot_his  = []
        self.ax_his      = []
        self.ay_his      = []
        self.roll_his    = []
        self.pitch_his   = []
        
        # time stamp
        self.t0          = t0
        self.time_his    = []

        # Time for yawDot integration
        self.curr_time = rospy.get_rostime().to_sec()
        self.prev_time = self.curr_time

    def imu_callback(self,data):
        """Unpack message from sensor, IMU"""
        
        self.curr_time = rospy.get_rostime().to_sec()

        if self.prev_time > 0:
            self.yaw += self.psiDot * (self.curr_time-self.prev_time)
   
        ori = data.orientation
        quaternion = (ori.x, ori.y, ori.z, ori.w)
        (roll_raw, pitch_raw, dummy) = transformations.euler_from_quaternion(quaternion)
        self.roll   = roll_raw
        self.pitch  = pitch_raw

        w_z = data.angular_velocity.z
        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        a_z = data.linear_acceleration.z

        self.psiDot = w_z
        # Transformation from imu frame to vehicle frame (negative roll/pitch and reversed matrix multiplication to go back)
        self.ax = cos(-pitch_raw)*a_x + sin(-pitch_raw)*sin(-roll_raw)*a_y - sin(-pitch_raw)*cos(-roll_raw)*a_z
        self.ay = cos(-roll_raw)*a_y + sin(-roll_raw)*a_z

        self.prev_time = self.curr_time

        self.saveHistory()

    def saveHistory(self):
        """ Save measurement data into history array"""

        self.time_his.append(self.curr_time)
        
        self.yaw_his.append(self.yaw)
        self.psiDot_his.append(self.psiDot)
        self.ax_his.append(self.ax)
        self.ay_his.append(self.ay)
        self.roll_his.append(self.roll)
        self.pitch_his.append(self.pitch)



class GpsClass(object):
    """ Object collecting GPS measurement data
    Attributes:
        Measurement:
            1.x 2.y
        Measurement history:
            1.x_his 2.y_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        # rospy.Subscriber('hedge_pos', hedge_pos, self.gps_callback, queue_size=1)
        rospy.Subscriber('hedge_imu_fusion', hedge_imu_fusion, self.gps_callback, queue_size=1)

        # GPS measurement
        self.x      = 0.0
        self.y      = 0.0
        
        # GPS measurement history
        self.x_his  = np.array([])
        self.y_his  = np.array([])
        
        # time stamp
        self.t0         = t0
        self.time_his   = np.array([])
        self.curr_time  = rospy.get_rostime().to_sec() 

    def gps_callback(self,data):
        """Unpack message from sensor, GPS"""
        self.curr_time = rospy.get_rostime().to_sec()

        self.x = data.x_m
        self.y = data.y_m

        # 1) x(t) ~ c0x + c1x * t + c2x * t^2
        # 2) y(t) ~ c0y + c1y * t + c2y * t^2
        # c_X = [c0x c1x c2x] and c_Y = [c0y c1y c2y] 
        # n_intplt = 20
        # if size(self.x_his,0) > n_intplt: # do interpolation when there is enough points
        #     x_intplt = self.x_his[-n_intplt:]
        #     y_intplt = self.y_his[-n_intplt:]
        #     t_intplt = self.time_his[-n_intplt:]
        #     t_matrix = vstack([t_intplt**2, t_intplt, ones(sz)]).T
        #     self.c_X = linalg.lstsq(t_matrix, x_intplt)[0]
        #     self.c_Y = linalg.lstsq(t_matrix, y_intplt)[0]
        #     self.x = polyval(self.c_X, self.curr_time)
        #     self.y = polyval(self.c_Y, self.curr_time)

        self.saveHistory()

    def saveHistory(self):
        self.time_his = np.append(self.time_his,self.curr_time)

        self.x_his = np.append(self.x_his,self.x)
        self.y_his = np.append(self.y_his,self.y)


class EncClass(object):
    """ Object collecting ENC measurement data
    Attributes:
        Measurement:
            1.v_fl 2.v_fr 3. v_rl 4. v_rr
        Measurement history:
            1.v_fl_his 2.v_fr_his 3. v_rl_his 4. v_rr_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        rospy.Subscriber('vel_est', Vel_est, self.enc_callback, queue_size=1)

        # ENC measurement
        self.v_fl      = 0.0
        self.v_fr      = 0.0
        self.v_rl      = 0.0
        self.v_rr      = 0.0
        self.v_meas    = 0.0
        
        # ENC measurement history
        self.v_fl_his    = []
        self.v_fr_his    = []
        self.v_rl_his    = []
        self.v_rr_his    = []
        self.v_meas_his  = []
        
        # time stamp
        self.v_count    = 0
        self.v_prev     = 0.0
        self.t0         = t0
        self.time_his   = []
        self.curr_time  = rospy.get_rostime().to_sec()

    def enc_callback(self,data):
        """Unpack message from sensor, ENC"""
        self.curr_time = rospy.get_rostime().to_sec()

        self.v_fl = data.vel_fl
        self.v_fr = data.vel_fr
        self.v_rl = data.vel_bl
        self.v_rr = data.vel_br
        v_est = self.v_rr
        if v_est != self.v_prev:
            self.v_meas = v_est
            self.v_prev = v_est
            self.v_count = 0
        else:
            self.v_count += 1
            if self.v_count > 10:     # if 10 times in a row the same measurement
                self.v_meas = 0       # set velocity measurement to zero

        self.saveHistory()

    def saveHistory(self):
        self.time_his.append(self.curr_time)
        
        self.v_fl_his.append(self.v_fl)
        self.v_fr_his.append(self.v_fr)
        self.v_rl_his.append(self.v_rl)
        self.v_rr_his.append(self.v_rr)

        self.v_meas_his.append(self.v_meas)

class EcuClass(object):
    """ Object collecting CMD command data
    Attributes:
        Input command:
            1.a 2.df
        Input command history:
            1.a_his 2.df_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        rospy.Subscriber('ecu', ECU, self.ecu_callback, queue_size=1)

        # ECU measurement
        self.a  = 0.0
        self.df = 0.0
        
        # ECU measurement history
        self.a_his  = []
        self.df_his = []
        
        # time stamp
        self.t0         = t0
        self.time_his   = []
        self.curr_time  = rospy.get_rostime().to_sec()

    def ecu_callback(self,data):
        """Unpack message from sensor, ECU"""
        self.curr_time = rospy.get_rostime().to_sec()

        self.a  = data.motor
        self.df = data.servo

        self.saveHistory()

    def saveHistory(self):
        self.time_his.append(self.curr_time)
        
        self.a_his.append(self.a)
        self.df_his.append(self.df)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass