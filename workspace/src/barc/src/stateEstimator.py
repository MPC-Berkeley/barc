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
from marvelmind_nav.msg import hedge_imu_fusion
from std_msgs.msg import Header
from numpy import eye, zeros, diag, tan, cos, sin, vstack, linalg, pi
from numpy import ones, polyval, size, dot, add
from scipy.linalg import inv, cholesky
from tf import transformations
from trackInitialization import Map
import math
import numpy as np
import os
import pdb

def main():
    # node initialization
    rospy.init_node("state_estimation")
    a_delay     = rospy.get_param("state_estimator/delay_a")
    df_delay    = rospy.get_param("state_estimator/delay_df")
    loop_rate   = 50.0
   
    # Initialize Map
    map = Map()

    Q_noVy = eye(8)
    Q_noVy[0,0] = 0.01 # x
    Q_noVy[1,1] = 0.01 # y
    Q_noVy[2,2] = 0.01 # vx
    Q_noVy[3,3] = 0.01 # vy
    Q_noVy[4,4] = 1.0 # ax
    Q_noVy[5,5] = 1.0 # ay
    Q_noVy[6,6] = 0.01 # psi
    Q_noVy[7,7] = 1.0 # psidot
    # Q[8,8] = 0.0 # psiDot in the model
    R_noVy = eye(6)
    R_noVy[0,0] = 1.0   # x
    R_noVy[1,1] = 1.0   # y
    R_noVy[2,2] = 0.1   # vx
    R_noVy[3,3] = 10.0   # ax
    R_noVy[4,4] = 10.0   # ay
    R_noVy[5,5] = 0.001    # psiDot

    Q = eye(8)
    Q[0,0] = 0.01
    Q[1,1] = 0.01
    Q[2,2] = 0.5
    Q[3,3] = 0.5
    Q[4,4] = 1.0
    Q[5,5] = 1.0
    Q[6,6] = 0.01
    Q[7,7] = 10.0
    R = eye(7)
    R[0,0] = 0.0005
    R[1,1] = 0.0005
    R[2,2] = 0.1
    R[3,3] = 0.01
    R[4,4] = 10.0
    R[5,5] = 20.0
    R[6,6] = 0.001
    thReset = 1.4

    Q_1 = eye(8)
    Q_1[0,0] = 0.01
    Q_1[1,1] = 0.01
    Q_1[2,2] = 0.5
    Q_1[3,3] = 0.5
    Q_1[4,4] = 1.0
    Q_1[5,5] = 1.0
    Q_1[6,6] = 0.01
    Q_1[7,7] = 10.0
    R_1 = eye(7)
    R_1[0,0] = 5.0
    R_1[1,1] = 5.0
    R_1[2,2] = 1.0
    R_1[3,3] = 0.0001
    R_1[4,4] = 10.0
    R_1[5,5] = 20.0
    R_1[6,6] = 0.001
    thReset_1 = 0.4

    twoFilters = True


    t0 = rospy.get_rostime().to_sec()
    imu = ImuClass(t0)
    gps = GpsClass(t0)
    enc = EncClass(t0)
    ecu = EcuClass(t0)

    est_noVy = Estimator_noVy(t0,loop_rate,a_delay,df_delay,Q_noVy,R_noVy)

    est = Estimator(t0,loop_rate,a_delay,df_delay,Q,R, thReset)
    est_1 = Estimator(t0,loop_rate,a_delay,df_delay,Q_1,R_1, thReset_1)

    estMsg = pos_info()
    
    while not rospy.is_shutdown():

        est_noVy.estimateState(imu,gps,enc,ecu,est_noVy.ekf)
        
        est.estimateState(imu,gps,enc,ecu,est.ekf)
        
        est_1.estimateState(imu,gps,enc,ecu,est_1.ekf)

        estMsg.s, estMsg.ey, estMsg.epsi, _ = map.getLocalPosition(est.x_est, est.y_est, est.yaw_est)
        
        estMsg.header.stamp = rospy.get_rostime()
        
        if twoFilters == True:
            estMsg.v        = np.sqrt(est.vx_est**2 + est.vy_est**2)

            estMsg.x        = est.x_est 
            estMsg.y        = est.y_est

            estMsg.v_x      = est_1.vx_est 
            estMsg.v_y      = est_1.vy_est

            estMsg.psi      = est_1.yaw_est
            estMsg.psiDot   = est_1.psiDot_est

            estMsg.a_x      = est_1.ax_est
            estMsg.a_y      = est_1.ay_est

            estMsg.u_a      = ecu.a
            estMsg.u_df     = ecu.df
        else:
            estMsg.v        = np.sqrt(est_noVy.vx_est**2 + est_noVy.vy_est**2)

            estMsg.x        = est_noVy.x_est 
            estMsg.y        = est_noVy.y_est

            estMsg.v_x      = est_noVy.vx_est 
            estMsg.v_y      = est_noVy.vy_est

            estMsg.psi      = est_noVy.yaw_est
            estMsg.psiDot   = est_noVy.psiDot_est

            estMsg.a_x      = est_noVy.ax_est
            estMsg.a_y      = est_noVy.ay_est

            estMsg.u_a      = ecu.a
            estMsg.u_df     = ecu.df




        est.state_pub_pos.publish(estMsg)

        est.saveHistory()
        est.rate.sleep()

    homedir = os.path.expanduser("~")
    pathSave = os.path.join(homedir,"barc_debugging/estimator_output.npz")
    np.savez(pathSave,yaw_est_his       = est.yaw_est_his,
                      psiDot_est_his    = est.psiDot_est_his,
                      x_est_his         = est.x_est_his,
                      y_est_his         = est.y_est_his,
                      vx_est_his        = est.vx_est_his,
                      vy_est_his        = est.vy_est_his,
                      ax_est_his        = est.ax_est_his,
                      ay_est_his        = est.ay_est_his,
                      estimator_time    = est.time_his)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_imu.npz")
    np.savez(pathSave,psiDot_his    = imu.psiDot_his,
                      roll_his      = imu.roll_his,
                      pitch_his     = imu.pitch_his,
                      yaw_his       = imu.yaw_his,
                      ax_his        = imu.ax_his,
                      ay_his        = imu.ay_his,
                      imu_time      = imu.time_his)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_gps.npz")
    np.savez(pathSave,x_his         = gps.x_his,
                      y_his         = gps.y_his,
                      gps_time      = gps.time_his)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_enc.npz")
    np.savez(pathSave,v_fl_his          = enc.v_fl_his,
                      v_fr_his          = enc.v_fr_his,
                      v_rl_his          = enc.v_rl_his,
                      v_rr_his          = enc.v_rr_his,
                      enc_time          = enc.time_his)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_ecu.npz")
    np.savez(pathSave,a_his         = ecu.a_his,
                      df_his        = ecu.df_his,
                      ecu_time      = ecu.time_his)

    print "Finishing saveing state estimation data"

class Estimator_noVy(object):
    def __init__(self,t0,loop_rate,a_delay,df_delay,Q,R):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        dt          = 1.0 / loop_rate
        L_f         = 0.125       # distance from CoG to front axel
        L_r         = 0.125       # distance from CoG to rear axel
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
        self.psiDot_drift_est = 0.0

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

    # ecu command update
    def estimateState(self,imu,gps,enc,ecu,KF):
        self.a_his.append(ecu.a)
        self.df_his.append(ecu.df)
        u = [self.a_his.pop(0), self.df_his.pop(0)]
        # u = [ecu.a, self.df_his.pop(0)]
        
        bta = 0.5 * u[1]
        y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot, sin(bta)*enc.v_meas])
        y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot])

        KF(y,u)

    def ekf(self, y, u):
        
        xDim    = self.z.size                           # dimension of the state
        mx_kp1  = self.f(self.z, u)                     # predict next state
        A       = self.numerical_jac(self.f, self.z, u) # linearize process model about current state
        P_kp1   = dot(dot(A,self.P),A.T) + self.Q           # proprogate variance
        my_kp1  = self.h(mx_kp1, u)                              # predict future output
        H       = self.numerical_jac(self.h, mx_kp1, u)     # linearize measurement model about predicted next state
        P12     = dot(P_kp1, H.T)                           # cross covariance
        K       = dot(P12, inv( dot(H,P12) + self.R))       # Kalman filter gain
        

        self.z  = mx_kp1 + dot(K,(y - my_kp1))
        if np.abs(y[5]) < 0.5:
            self.z[3] = 0

        self.P  = dot(dot(K,self.R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )

        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z


    def ukf(self, y, u):
       
        xDim        = self.z.size
        sqrtnP      = cholesky(xDim*self.P)
        sm_km1      = list(add(self.z,sqrtnP))
        sm_km1.extend(list(add(self.z,-sqrtnP)))

        # prior update
        sx_k = [self.f(s, u) for s in sm_km1]
        mx_k = 1.0/len(sx_k)*sum(sx_k)
        P_m  = self.Q + 1.0/len(sx_k)*sum([np.outer((sx-mx_k),(sx-mx_k)) for sx in sx_k])

        # posterior update
        sy_k = [self.h(s, u) for s in sx_k]
        my_k = 1.0/len(sy_k)*sum(sy_k)
        P_zz  = self.R + 1.0/len(sy_k)*sum([np.outer((sy-my_k),(sy-my_k)) for sy in sy_k])

        # cross covariance
        P_xz = 1.0/len(sy_k)*sum([np.outer((sx_k[i]-mx_k),(sy_k[i]-my_k)) for i in range(len(sy_k))])

        # Kalman filter
        K = dot(P_xz,inv(P_zz))
        self.z = mx_k + dot(K, y-my_k)
        self.P = P_m - dot(K, dot(P_zz, K.T))
        
        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z

    def numerical_jac(self,func,x,u):
       
        y = func(x,u)
        
        jac = zeros( (y.size,x.size) )
        eps = 1e-5
        xp = np.copy(x)
        
        for i in range(x.size):
            xp[i] = x[i] + eps/2.0
            yhi = func(xp, u)
            xp[i] = x[i] - eps/2.0
            ylo = func(xp, u)
            xp[i] = x[i]
            jac[:,i] = (yhi - ylo) / eps
        return jac

    def f(self, z, u):
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
        # zNext[8] = z[8]                                         # psidot_drift
        return np.array(zNext)

    def h(self, x, u):
        """ This is the measurement model to the kinematic<->sensor model above """
        y = [0]*6
        y[0] = x[0]      # x
        y[1] = x[1]      # y
        y[2] = x[2]      # vx
        y[3] = x[4]      # a_x
        y[4] = x[5]      # a_y
        # y[5] = x[7]+x[8] # psiDot
        y[5] = x[7] # psiDot
        # y[6] = x[3]      # vy
        return np.array(y)

    def saveHistory(self):

        self.x_est_his.append(self.x_est)
        self.y_est_his.append(self.y_est)
        self.vx_est_his.append(self.vx_est)
        self.vy_est_his.append(self.vy_est)
        self.v_est_his.append(self.v_est)
        self.ax_est_his.append(self.ax_est)
        self.ay_est_his.append(self.ay_est)
        self.yaw_est_his.append(self.yaw_est)
        self.psiDot_est_his.append(self.psiDot_est)

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
        if rospy.get_param("feature_flag"): self.z[6]   = pi/4
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

        self.oldGPS_x = 0.0
        self.oldGPS_y = 0.0

    # ecu command update
    def estimateState(self,imu,gps,enc,ecu,KF):
        """Do extended Kalman filter to estimate states"""
        self.curr_time = rospy.get_rostime().to_sec()

        self.a_his.append(ecu.a)
        self.df_his.append(ecu.df)
        u = [self.a_his.pop(0), self.df_his.pop(0)]
        # u = [ecu.a, self.df_his.pop(0)]
        
        bta = 0.5 * u[1]

        dist   = np.sqrt(( self.x_est - gps.x )**2 + ( self.y_est - gps.y )**2)

        # if ( dist >= 1 ) or ( (gps.x == self.oldGPS_x) and (gps.x == self.oldGPS_y) ):
        # if ( (gps.x == self.oldGPS_x) and (gps.x == self.oldGPS_y) ):
        if 0 == 1:
            modeGPS = False
            y = np.array([enc.v_meas, imu.ax, imu.ay, imu.psiDot, bta * enc.v_meas])
        else:
            modeGPS = True
            y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot, bta * enc.v_meas])

        self.oldGPS_x = gps.x
        self.oldGPS_y = gps.y

        gps.x_his = np.append(gps.x_his,y[0])
        gps.y_his = np.append(gps.y_his,y[1])
        enc.v_fl_his.append(enc.v_meas)
        enc.v_fr_his.append(enc.v_meas)
        enc.v_rl_his.append(enc.v_meas)
        enc.v_rr_his.append(enc.v_meas)
        enc.v_meas_his.append(enc.v_meas)
        imu.ax_his.append(imu.ax)
        imu.ay_his.append(imu.ay)
        imu.psiDot_his.append(imu.psiDot)
        ecu.a_his.append(u[0])
        ecu.df_his.append(u[1])


        if np.abs(imu.psiDot) < self.thReset:
            self.z[3] = 0

        KF(y,u, modeGPS)


    def ekf(self, y, u, modeGPS):
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
        
        xDim    = self.z.size                           # dimension of the state

        mx_kp1  = self.f(self.z, u)                               # predict next state
        A       = self.numerical_jac(self.f, self.z, u, modeGPS)  # linearize process model about current state

        P_kp1   = dot(dot(A,self.P),A.T) + self.Q                 # proprogate variance

        my_kp1  = self.h(mx_kp1, u, modeGPS)                      # predict future output
        H       = self.numerical_jac(self.h, mx_kp1, u, modeGPS)  # linearize measurement model about predicted next state
        
        P12     = dot(P_kp1, H.T)                                 # cross covariance

        if modeGPS == True:
            K       = dot(P12, inv( dot(H,P12) + self.R))       # Kalman filter gain
        else:
            K       = dot(P12, inv( dot(H,P12) + self.R[2:,2:]))       # Kalman filter gain
            
        self.z  = mx_kp1 + dot(K,(y - my_kp1))

        if modeGPS == True:
            self.P  = dot(dot(K,self.R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )
        else:
            self.P  = dot(dot(K,self.R[2:,2:]),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )



        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z


    def ukf(self, y, u):
        """
        UKF   Unscented Kalman Filter for nonlinear dynamic systems
        ekf(f,mx,P,h,z,Q,R) returns state estimate, x and state covariance, P 
        for nonlinear dynamic system:
                  x[k] = f(x[k-1],u[k-1]) + v[k-1]
                  y[k] = h(x[k]) + w[k]
        where v ~ N(0,Q) meaning v is gaussian noise with covariance Q
              w ~ N(0,R) meaning w is gaussian noise with covariance R
        Inputs:    f: function handle for f(x)
                   h: function handle for h(x)
                   y: current measurement
                   Q: process noise covariance 
                   R: measurement noise covariance
        Output:    mx_k: "a posteriori" state estimate
                   P_k: "a posteriori" state covariance
                   
        Notation: mx_k = E[x_k] and my_k = E[y_k], where m stands for "mean of"
        """

        # sigma-points: generate a list, "sm_km1"
        xDim        = self.z.size
        sqrtnP      = cholesky(xDim*self.P)
        sm_km1      = list(add(self.z,sqrtnP))
        sm_km1.extend(list(add(self.z,-sqrtnP)))

        # prior update
        sx_k = [self.f(s, u) for s in sm_km1]
        mx_k = 1.0/len(sx_k)*sum(sx_k)
        P_m  = self.Q + 1.0/len(sx_k)*sum([np.outer((sx-mx_k),(sx-mx_k)) for sx in sx_k])

        # posterior update
        sy_k = [self.h(s, u) for s in sx_k]
        my_k = 1.0/len(sy_k)*sum(sy_k)
        P_zz  = self.R + 1.0/len(sy_k)*sum([np.outer((sy-my_k),(sy-my_k)) for sy in sy_k])

        # cross covariance
        P_xz = 1.0/len(sy_k)*sum([np.outer((sx_k[i]-mx_k),(sy_k[i]-my_k)) for i in range(len(sy_k))])

        # Kalman filter
        K = dot(P_xz,inv(P_zz))
        self.z = mx_k + dot(K, y-my_k)
        self.P = P_m - dot(K, dot(P_zz, K.T))
        
        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z

    def numerical_jac(self,func,x,u, modeGPS):
        """
        Function to compute the numerical jacobian of a vector valued function 
        using final differences
        """
        # numerical gradient and diagonal hessian
        y = func(x,u, modeGPS)
        
        jac = zeros( (y.size,x.size) )
        eps = 1e-5
        xp = np.copy(x)
        
        for i in range(x.size):
            xp[i] = x[i] + eps/2.0
            yhi = func(xp, u, modeGPS)
            xp[i] = x[i] - eps/2.0
            ylo = func(xp, u, modeGPS)
            xp[i] = x[i]
            jac[:,i] = (yhi - ylo) / eps
        return jac

    def f(self, z, u, modeGPS=True):
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
        return np.array(zNext)

    def h(self, x, u, modeGPS):
        """ This is the measurement model to the kinematic<->sensor model above """
        if modeGPS:
            y = [0]*7
            y[0] = x[0]   # x
            y[1] = x[1]   # y
            y[2] = x[2]   # vx
            y[3] = x[4]   # a_x
            y[4] = x[5]   # a_y
            y[5] = x[7]   # psiDot
            y[6] = x[3]   # vy
        else:
            y = [0]*5
            y[0] = x[2]   # vx
            y[1] = x[4]   # a_x
            y[2] = x[5]   # a_y
            y[3] = x[7]   # psiDot
            y[4] = x[3]   # vy
        return np.array(y)

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
        v_est = (self.v_rl + self.v_rr)/2
        if v_est != self.v_prev:
            self.v_meas = v_est
            self.v_prev = v_est
            self.v_count = 0
        else:
            self.v_count += 1
            if self.v_count > 10:     # if 10 times in a row the same measurement
                self.v_meas = 0       # set velocity measurement to zero

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

    def saveHistory(self):
        self.time_his.append(self.curr_time)
        
        self.a_his.append(self.a)
        self.df_his.append(self.df)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass