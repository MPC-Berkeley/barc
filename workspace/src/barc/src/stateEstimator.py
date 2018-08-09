#!/usr/bin/env python
"""
    File name: stateEstimator.py
    Author: Shuqi Xu
    Email: shuqixu@berkeley.edu (xushuqi8787@gmail.com)
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

import rospy
import os
import sys
from datetime import datetime
homedir = os.path.expanduser("~")
sys.path.append(os.path.join(homedir,"barc/workspace/src/barc/src/library"))
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

def srvOutput2Angle(fbk_srv):
    # angle_rad =  -0.0033597966955940897 *fbk_srv +  1.0990602157436302
    angle_rad =  -0.003369111897931554 *fbk_srv +  1.1421487960691172
    return angle_rad



def main():
    # node initialization
    rospy.init_node("state_estimation")
    a_delay     = 0.0
    df_delay    = 0.0
    loop_rate   = 100.0
   	
    # Tuning for estimator at high speed
    Q_hs = eye(7)
    Q_hs[0,0]  =  rospy.get_param("/state_estimator/Qx_hs") # 0.5     # x
    Q_hs[1,1]  =  rospy.get_param("/state_estimator/Qy_hs") # 0.5     # y
    Q_hs[2,2]  =  rospy.get_param("/state_estimator/Qvx_hs") #10.0     # vx
    Q_hs[3,3]  =  rospy.get_param("/state_estimator/Qvy_hs") #10.0     # vy
    Q_hs[4,4]  =  rospy.get_param("/state_estimator/Qax_hs") #1.0      # ax
    Q_hs[5,5]  =  rospy.get_param("/state_estimator/Qay_hs") #1.0      # ay 
    Q_hs[6,6]  =  rospy.get_param("/state_estimator/Qpsi_hs") #10 + 80.0      # psi
    R_hs = eye(6)
    R_hs[0,0]  = rospy.get_param("/state_estimator/Rx_hs")      # 10 + 40.0      # x
    R_hs[1,1]  = rospy.get_param("/state_estimator/Ry_hs")      #10 + 40.0      # y
    R_hs[2,2]  = rospy.get_param("/state_estimator/Rvx_hs")     # 0.1      # vx
    R_hs[3,3]  = rospy.get_param("/state_estimator/Rax_hs")     #30 + 10.0      # ax 
    R_hs[4,4]  = rospy.get_param("/state_estimator/Ray_hs")     #40.0      # ay 
    R_hs[5,5]  = rospy.get_param("/state_estimator/Rvy_hs")     # 0.01    # vy

    # Tuning for estimator at low speed
    Q_ls = eye(7)
    Q_ls[0,0]  =  rospy.get_param("/state_estimator/Qx_ls") # 0.5     # x
    Q_ls[1,1]  =  rospy.get_param("/state_estimator/Qy_ls") #0.5     # y
    Q_ls[2,2]  =  rospy.get_param("/state_estimator/Qvx_ls") #10.0     # vx
    Q_ls[3,3]  =  rospy.get_param("/state_estimator/Qvy_ls") #10.0     # vy
    Q_ls[4,4]  =  rospy.get_param("/state_estimator/Qax_ls") #1.0      # ax
    Q_ls[5,5]  =  rospy.get_param("/state_estimator/Qay_ls") #1.0      # ay 
    Q_ls[6,6]  =  rospy.get_param("/state_estimator/Qpsi_ls") #10 + 80.0      # psi
    R_ls = eye(6)
    R_ls[0,0]  = rospy.get_param("/state_estimator/Rx_ls")       # 10 + 40.0      # x
    R_ls[1,1]  = rospy.get_param("/state_estimator/Ry_ls")       # 10 + 40.0      # y
    R_ls[2,2]  = rospy.get_param("/state_estimator/Rvx_ls")      # 0.1      # vx
    R_ls[3,3]  = rospy.get_param("/state_estimator/Rax_ls")      # 30 + 10.0      # ax 
    R_ls[4,4]  = rospy.get_param("/state_estimator/Ray_ls")      # 40.0      # ay 
    R_ls[5,5]  = rospy.get_param("/state_estimator/Rvy_ls")      #  0.01    # vy    

    thReset      = rospy.get_param("/state_estimator/thReset")       # 0.4
    vSwitch      = rospy.get_param("/state_estimator/vSwitch")       # 1.0
    psiSwitch    = rospy.get_param("/state_estimator/psiSwitch")       # 0.5 * 2.0


    # Q = eye(7)
    # Q[0,0] = 0.001**2 	# Q_x
    # Q[1,1] = 0.001**2 	# Q_y
    # Q[2,2] = 0.01**2 	# Q_vx
    # Q[3,3] = 0.001**2 	# Q_vy
    # Q[4,4] = 0.1**2 	# Q_ax
    # Q[5,5] = 0.1**2 	# Q_ay
    # Q[6,6] = 0.001**2   # Q_psi
    # R = eye(6)
    # R[0,0] = 10.0#**2 	# R_x
    # R[1,1] = 10.0#**2 	# R_y
    # R[2,2] = 0.01**2    # R_vx
    # R[3,3] = 0.016 	    # R_ax
    # R[4,4] = 0.024 	    # R_ay
    # R[5,5] = 0.001**2   # R_vy

    # Q_hs = Q
    # R_ls = R
    # Q_hs = Q
    # R_ls = R

    t0 = rospy.get_rostime().to_sec()
    imu = ImuClass(t0)
    gps = GpsClass(t0)
    enc = EncClass(t0)
    ecu = EcuClass(t0)
    est = Estimator(t0,loop_rate,a_delay,df_delay,Q_hs,Q_ls,R_hs,R_ls,thReset,vSwitch,psiSwitch)
    fbk_srv = fbServoClass()

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
        # if ecu.a != 0:
        est.estimateState(imu,gps,enc,ecu,est.ekf)
        # Save estimator Input.
        saved_x_est.append(estMsg.x)
        saved_y_est.append(estMsg.y)
        saved_vx_est.append(estMsg.v_x)
        saved_vy_est.append(estMsg.v_y)
        saved_psi_est.append(estMsg.psi)
        saved_psiDot_est.append(estMsg.psiDot)
        saved_ax_est.append(estMsg.a_x)
        saved_ay_est.append(estMsg.a_y)
        saved_switch.append(int(est.flagVy))

        # Save estimator output
        est.saveHistory()

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
        estMsg.u_df     = srvOutput2Angle(fbk_srv.value)
        est.state_pub_pos.publish(estMsg)


        est.rate.sleep()


    print "gps x      package lost:", float(est.x_count)/float(est.pkg_count)
    print "gps y      package lost:", float(est.y_count)/float(est.pkg_count)
    print "enc v      package lost:", float(est.v_meas_count)/float(est.pkg_count)
    print "imu ax     package lost:", float(est.ax_count)/float(est.pkg_count)
    print "imu ay     package lost:", float(est.ay_count)/float(est.pkg_count)
    print "imu psiDot package lost:", float(est.psiDot_count)/float(est.pkg_count)
    
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
                      flagVy            = saved_switch,
                      roll_his          = est.roll_his,
                      pitch_his         = est.pitch_his,
                      wx_his            = est.wx_his,
                      wy_his            = est.wy_his,
                      wz_his            = est.wz_his,
                      v_rl_his          = est.v_rl_his,
                      v_rr_his          = est.v_rr_his,
                      v_fl_his          = est.v_fl_his,
                      v_fr_his          = est.v_fr_his,
                      psi_raw_his       = est.psi_raw_his)

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
    def __init__(self,t0,loop_rate,a_delay,df_delay,Q_hs,Q_ls,R_hs,R_ls,thReset,vSwitch,psiSwitch):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        dt             = 1.0 / loop_rate
        self.rate      = rospy.Rate(loop_rate)
        L_f            = rospy.get_param("L_a")       # distance from CoG to front axel
        L_r            = rospy.get_param("L_b")       # distance from CoG to rear axel
        self.vhMdl     = (L_f, L_r)
        self.Q_hs      = Q_hs
        self.Q_ls      = Q_ls
        self.R_hs      = R_hs
        self.R_ls      = R_ls
        self.thReset   = thReset
        self.vSwitch   = vSwitch
        self.psiSwitch = psiSwitch
        self.P         = np.eye(np.size(Q_hs,0)) # initializationtial covariance matrix
        self.z         = np.zeros(np.size(Q_hs,0)) # initial state mean
        self.dt        = dt
        self.a_delay        = a_delay
        self.df_delay       = df_delay
        self.motor_his      = [0.0]*int(a_delay/dt)
        self.servo_his      = [0.0]*int(df_delay/dt)

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
        self.psiDot_drift_est = 0.0
        self.curr_time      = rospy.get_rostime().to_sec() - self.t0

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
        self.x_his       = []
        self.y_his       = []
        self.v_meas_his  = []
        self.vy_meas_his = []
        self.ax_his      = []
        self.ay_his      = []
        self.psiDot_his  = []
        self.inp_a_his   = []
        self.inp_df_his  = []
        self.psi_raw_his = []

        # Angular Velocities
        self.wx_his     = []
        self.wy_his     = []
        self.wz_his     = []
        
        # Roll an pitch
        self.roll_his   = []
        self.pitch_his  = []

        # Encored Readinds
        self.v_rl_his   =[]
        self.v_rr_his   =[]
        self.v_fl_his   =[]
        self.v_fr_his   =[]

        # COUNTERS FOR PACKAGE LOST
        self.pkg_count    = 0
        self.x_count      = 0
        self.y_count      = 0
        self.v_meas_count = 0
        self.ax_count     = 0
        self.ay_count     = 0
        self.psiDot_count = 0

        self.gps_time = []
        self.enc_time = []
        self.imu_time = []

    # ecu command update
    def estimateState(self,imu,gps,enc,ecu,KF):
        """Do extended Kalman filter to estimate states"""
        self.curr_time = rospy.get_rostime().to_sec() - self.t0

        self.motor_his.append(ecu.a)
        self.servo_his.append(ecu.df)
        u = [self.motor_his.pop(0), self.servo_his.pop(0), imu.psiDot]
        
        y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, 0.5*u[1]*enc.v_meas])

        # Read Measurements which are not used but must be saved
        wx_imu    = imu.w_x
        wy_imu    = imu.w_y
        wz_imu    = imu.w_z
        roll_imu  = imu.roll
        pitch_imu = imu.pitch
        psiRaw    = imu.yaw_raw
        
        v_rl_enc = enc.v_rl
        v_rr_enc = enc.v_rr
        v_fl_enc = enc.v_fl
        v_fr_enc = enc.v_fr

        KF(y,u)

        # SAVE THE measurement/input SEQUENCE USED BY KF
        self.x_his.append(y[0])
        self.y_his.append(y[1])
        self.v_meas_his.append(y[2])
        self.ax_his.append(y[3])
        self.ay_his.append(y[4])
        self.vy_meas_his.append(y[5])
        self.inp_a_his.append(u[0])
        self.inp_df_his.append(u[1])
        self.psiDot_his.append(u[2])
        self.psi_raw_his.append(psiRaw)

        # Angular Velocities
        self.wx_his.append(wx_imu)
        self.wy_his.append(wy_imu)
        self.wz_his.append(wz_imu)
        
        # Roll an pitch
        self.roll_his.append(roll_imu)
        self.pitch_his.append(pitch_imu)

        # Encored Readinds
        self.v_rl_his.append(v_rl_enc)
        self.v_rr_his.append(v_rr_enc)
        self.v_fl_his.append(v_fl_enc)
        self.v_fr_his.append(v_fr_enc)
        
        self.gps_time.append(gps.curr_time)
        self.imu_time.append(imu.curr_time)
        self.enc_time.append(enc.curr_time)
    def ekf(self, y, u):
        
        idx = []
        if u[0]!=0: # start the package loss counting when the car start moving
            if self.x_his[-1] == y[0] and self.y_his[-1] == y[1]:
                # MultiRate for gps
                idx.append(0)
                idx.append(1)
                self.x_count += 1
                self.y_count += 1
            if self.v_meas_his[-1] == y[2]:
                # MultiRate for encoder
                self.v_meas_count += 1
                idx.append(2)
                idx.append(5)
            if self.ax_his[-1] == y[3]:
                self.ax_count += 1
            if self.ay_his[-1] == y[4]:
                self.ay_count += 1
            if self.psiDot_his[-1] == u[2]:
                self.psiDot_count += 1
            self.pkg_count += 1

        # Decide is vy is used in the filter
        if (abs(y[2]) > self.vSwitch or abs(u[2]) > self.psiSwitch): # Vy reset 
            idx.append(5)
            Q = self.Q_hs
            R = self.R_hs
            self.flagVy = False

            # if abs(u[2]) < 0.9:
            #     Q[6] = 500 * self.Q_hs
        else:
            Q = self.Q_ls
            R = self.R_ls
            self.flagVy = True

        # Decide if vy has to be set to zero
        if abs(u[2]) < self.thReset: # was 0.4
            self.z[3]   = 0.0

        # Now do multirate KF
        # Prediction Step
        xDim    = self.z.size                               # dimension of the state
        mx_kp1  = self.f(self.z, u)                         # predict next state
        A       = self.numerical_jac(self.f, self.z, u)     # linearize process model about current state
        P_kp1   = dot(dot(A,self.P),A.T) + Q           # proprogate variance
        my_kp1  = self.h(mx_kp1, u)                         # predict future output
        H       = self.numerical_jac(self.h, mx_kp1, u)     # linearize measurement model about predicted next state

        # Measurement Update   
        H      = np.delete(H,(idx),axis=0)
        R      = np.delete(R,(idx),axis=0)
        R      = np.delete(R,(idx),axis=1)
        y      = np.delete(y,(idx),axis=0)      
        my_kp1 = np.delete(my_kp1,(idx),axis=0)
        P12    = dot(P_kp1, H.T)                      # cross covariance
        K      = dot(P12, inv( dot(H,P12) + R))       # Kalman filter gain
        self.z = mx_kp1 + dot(K,(y - my_kp1))

        self.P = dot(dot(K,R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )
        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est) = self.z
        self.psiDot_est     = u[2]

    def numerical_jac(self,func,x,u):
        """
        Function to compute the numerical jacobian of a vector valued function 
        using final differences
        """
        # numerical gradient and diagonal hessian
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
        zNext = [0]*7
        zNext[0] = z[0] + dt*(cos(z[6])*z[2] - sin(z[6])*z[3])  # x
        zNext[1] = z[1] + dt*(sin(z[6])*z[2] + cos(z[6])*z[3])  # y
        zNext[2] = z[2] + dt*(z[4]+u[2]*z[3])                   # v_x
        zNext[3] = z[3] + dt*(z[5]-u[2]*z[2])                   # v_y
        zNext[4] = z[4]                                         # a_x
        zNext[5] = z[5]                                         # a_y
        zNext[6] = z[6] + dt*(u[2])                             # psi
        return np.array(zNext)

    def h(self, x, u):
        """ This is the measurement model to the kinematic<->sensor model above """
        y = [0]*6
        y[0] = x[0]      # x
        y[1] = x[1]      # y
        y[2] = x[2]      # vx
        y[3] = x[4]      # a_x
        y[4] = x[5]      # a_y
        y[5] = x[3]      # vy
        # y[7] = x[6]+x[8] # psi_meas
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

# ==========================================================================================================================
# ==========================================================================================================================
# ==========================================================================================================================
# ==========================================================================================================================
class fbServoClass(object):
    def __init__(self):
        rospy.Subscriber('srv_fbk', ECU, self.srv_fbk_callback, queue_size=1)

        # ENC measurement
        self.recorded = [0.0]*int(10)
        self.value = 0.0

    def srv_fbk_callback(self,data):
        """Unpack message from sensor, ENC"""
        self.recorded.pop(0)
        self.recorded.append(data.servo)
        self.value = np.sum(self.recorded)/len(self.recorded)


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
        self.yaw_raw = 0.0
        self.w_x     = 0.0
        self.w_y     = 0.0
        self.w_z     = 0.0
        
        # Imu measurement history
        self.yaw_his     = [0.0]
        self.psiDot_his  = [0.0]
        self.ax_his      = [0.0]
        self.ay_his      = [0.0]
        self.roll_his    = [0.0]
        self.pitch_his   = [0.0]
        self.yaw_raw_his = [0.0]
        
        # time stamp
        self.t0          = t0
        self.time_his    = [0.0]

        # Time for yawDot integration
        self.curr_time = rospy.get_rostime().to_sec() - self.t0
        self.prev_time = self.curr_time

    def imu_callback(self,data):
        """Unpack message from sensor, IMU"""
        
        self.curr_time = rospy.get_rostime().to_sec() - self.t0
        if self.prev_time > 0:
            self.yaw += self.psiDot * (self.curr_time-self.prev_time)
        self.prev_time = self.curr_time
   
        ori = data.orientation
        quaternion = (ori.x, ori.y, ori.z, ori.w)
        (roll_raw, pitch_raw, yaw_raw) = transformations.euler_from_quaternion(quaternion)
        self.roll   = roll_raw
        self.pitch  = pitch_raw
        self.yaw_raw = np.unwrap([self.yaw_raw_his[-1],yaw_raw])[1]            

        w_z = data.angular_velocity.z
        w_x = data.angular_velocity.x
        w_y = data.angular_velocity.y
        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        a_z = data.linear_acceleration.z

        self.w_x = w_x
        self.w_y = w_y
        self.w_z = w_z


        # Ask Ugo's notes for transformation 

        # self.psiDot = w_z
        self.psiDot = sin(roll_raw) / cos(pitch_raw) * w_y + cos(roll_raw) / cos(pitch_raw) * w_z

        # self.ax = cos(-pitch_raw)*a_x + sin(-pitch_raw)*sin(-roll_raw)*a_y - sin(-pitch_raw)*cos(-roll_raw)*a_z
        self.ax = cos(pitch_raw)*a_x + sin(pitch_raw)*sin(roll_raw)*a_y + sin(pitch_raw)*cos(roll_raw)*a_z

        # self.ay = cos(-roll_raw)*a_y + sin(-roll_raw)*a_z
        self.ay = cos(roll_raw)*a_y - sin(roll_raw)*a_z


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
        self.yaw_raw_his.append(self.yaw_raw)

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

        # rospy.Subscriber('hedge_imu_fusion', hedge_imu_fusion, self.gps_callback, queue_size=1)
        rospy.Subscriber('hedge_pos', hedge_pos, self.gps_callback, queue_size=1)

        # GPS measurement
        self.angle  = 0.0
        self.x      = 0.0
        self.y      = 0.0
        self.x_ply  = 0.0
        self.y_ply  = 0.0
        
        # GPS measurement history
        self.angle_his  = np.array([0.0])
        self.x_his      = np.array([0.0])
        self.y_his      = np.array([0.0])
        self.x_ply_his  = np.array([0.0])
        self.y_ply_his  = np.array([0.0])
        
        # time stamp
        self.t0         = t0
        self.time_his   = np.array([0.0])
        self.time_ply_his = np.array([0.0])
        self.curr_time  = rospy.get_rostime().to_sec() - self.t0

    def gps_callback(self,data):
        """Unpack message from sensor, GPS"""
        self.curr_time = rospy.get_rostime().to_sec() - self.t0
        dist = np.sqrt((data.x_m-self.x_his[-1])**2+(data.y_m-self.y_his[-1])**2)
        # if dist < 0.5:
        # if self.x_his[-1] != data.x_m:
        self.x = data.x_m
        self.y = data.y_m            

        # 1) x(t) ~ c0x + c1x * t + c2x * t^2
        # 2) y(t) ~ c0y + c1y * t + c2y * t^2
        # c_X = [c0x c1x c2x] and c_Y = [c0y c1y c2y] 
        n_intplt = 50 # 50*0.01=0.5s data
        if size(self.x_ply_his,0) > n_intplt:
            x_intplt = self.x_ply_his[-n_intplt:]
            y_intplt = self.y_ply_his[-n_intplt:]
            t_intplt = self.time_ply_his[-n_intplt:]-self.time_ply_his[-n_intplt]
            t_matrix = vstack([t_intplt**2, t_intplt, ones(n_intplt)]).T
            c_X = linalg.lstsq(t_matrix, x_intplt)[0]
            c_Y = linalg.lstsq(t_matrix, y_intplt)[0]
            self.x_ply = polyval(c_X, self.curr_time-self.time_ply_his[-n_intplt])
            self.y_ply = polyval(c_Y, self.curr_time-self.time_ply_his[-n_intplt])

        # # Estimate yaw angle from previous 2 time steps
        # x_0 = self.x_his[-1]
        # y_0 = self.y_his[-1]
        # x_1 = self.x
        # y_1 = self.y
        # argument_y = y_1 - y_0
        # argument_x = x_1 - x_0
        # print "args", argument_x, argument_y
        # angle = 0.0
        # if argument_x > 0.0:
        #     angle = np.arctan(argument_y / argument_x)
        # if argument_y >= 0.0 and argument_x < 0.0:
        #     angle = np.pi + np.arctan(argument_y / argument_x)
        # if argument_y < 0.0 and argument_x < 0.0:
        #     angle = - np.pi + np.arctan(argument_y / argument_x)
        # if argument_y > 0.0 and argument_x == 0.0:
        #     angle = np.pi / 2.0
        # if argument_y < 0.0 and argument_x == 0.0:
        #     angle = - np.pi / 2.0
        # if angle < 0.0:
        #     angle += 2.0 * np.pi
        # self.angle = np.unwrap([self.angle_his[-1],angle])[1]

        self.saveHistory()

    def saveHistory(self):
        self.time_his = np.append(self.time_his,self.curr_time)
        self.angle_his= np.append(self.angle_his,self.angle)
        self.x_his    = np.append(self.x_his,self.x)
        self.y_his    = np.append(self.y_his,self.y)
        # if self.x_ply_his[-1] != self.x_ply:
        self.x_ply_his  = np.append(self.x_ply_his,self.x_ply)
        self.y_ply_his  = np.append(self.y_ply_his,self.y_ply)
        self.time_ply_his = np.append(self.time_ply_his,self.curr_time)

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
        self.curr_time  = rospy.get_rostime().to_sec() - self.t0

    def enc_callback(self,data):
        """Unpack message from sensor, ENC"""
        self.curr_time = rospy.get_rostime().to_sec() - self.t0

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
            if self.v_count > 40:
                self.v_meas = 0       

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
        self.curr_time  = rospy.get_rostime().to_sec() - self.t0

    def ecu_callback(self,data):
        """Unpack message from sensor, ECU"""
        self.curr_time = rospy.get_rostime().to_sec() - self.t0

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
