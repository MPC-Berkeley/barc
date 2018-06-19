"""
    File name: estimatorPlayBack.py
    Author: Shuqi Xu
    Email: shuqixu@berkeley.edu (xushuqi8787@gmail.com)
    Python Version: 2.7.12
"""
import os
import sys
homedir = os.path.expanduser("~")
sys.path.append(os.path.join(homedir,"barc/workspace/src/barc/src/library"))
from Localization_helpers import Track
from barc.msg import ECU, pos_info, Vel_est
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_imu_fusion
from std_msgs.msg import Header
from numpy import eye, zeros, diag, tan, cos, sin, vstack, linalg, pi
from numpy import ones, polyval, size, dot, add
from scipy.linalg import inv, cholesky
from tf import transformations
import matplotlib.pyplot as plt
import math
import numpy as np
import pdb

def main():
    playBack = PlayBack("06-18-16:06-sim")
    fig     = plt.figure("yaw")
    ax_yaw  = fig.add_subplot(1,1,1,ylabel="yaw_estimation")
    fig     = plt.figure("ax")
    ax_ax   = fig.add_subplot(1,1,1,ylabel="ax")
    fig     = plt.figure("ay")
    ax_ay   = fig.add_subplot(1,1,1,ylabel="ay")
    fig     = plt.figure("vx")
    ax_vx   = fig.add_subplot(1,1,1,ylabel="vx")
    fig     = plt.figure("vy")
    ax_vy   = fig.add_subplot(1,1,1,ylabel="vy")
    fig     = plt.figure("psiDot")
    ax_psiDot  = fig.add_subplot(1,1,1,ylabel="psiDot")
    fig = plt.figure("track x-y plot")
    ax_traj = fig.add_subplot(1,1,1,ylabel="track x-y plot")
    for i in [0,1,2,3,4]:
        playBack.replay(i)
        playBack.yawPlot(ax_yaw,i)
        playBack.axPlot(ax_ax,i)
        playBack.ayPlot(ax_ay,i)
        playBack.vxPlot(ax_vx,i)
        playBack.vyPlot(ax_vy,i)
        playBack.psiDotPlot(ax_psiDot,i)
        playBack.trajectoryPlot(ax_traj,i)
        playBack.playBackClean()
    plt.show()

class PlayBack(object):
    def __init__(self,folder_name):
        self.idx_min = 200
        self.idx_max = 250
        self.track = Track(0.01,1.0)
        self.track.createRaceTrack("MSC_lab")
        homedir = os.path.expanduser("~")
        pathSave = os.path.join(homedir,"barc_debugging/",folder_name,"estimator_output.npz")
        npz_output = np.load(pathSave)
        self.x_est_his           = npz_output["x_est_his"]
        self.y_est_his           = npz_output["y_est_his"]
        self.vx_est_his          = npz_output["vx_est_his"] 
        self.vy_est_his          = npz_output["vy_est_his"] 
        self.ax_est_his          = npz_output["ax_est_his"] 
        self.ay_est_his          = npz_output["ay_est_his"] 
        self.psiDot_est_his      = npz_output["psiDot_est_his"]  
        self.yaw_est_his         = npz_output["yaw_est_his"]
        self.KF_x_his            = npz_output["KF_x_his"]
        self.KF_y_his            = npz_output["KF_y_his"]
        self.KF_v_meas_his       = npz_output["KF_v_meas_his"]
        self.KF_ax_his           = npz_output["KF_ax_his"]
        self.KF_ay_his           = npz_output["KF_ay_his"]
        self.KF_psiDot_his       = npz_output["KF_psiDot_his"]
        self.KF_a_his            = npz_output["KF_a_his"]
        self.KF_df_his           = npz_output["KF_df_his"]
        self.estimator_time      = npz_output["estimator_time"]
        self.Q                   = npz_output["Q"]
        print "Q:", diag(self.Q)
        self.R                   = npz_output["R"]
        print "R:", diag(self.R)
        print "Finish loading data from", pathSave

        self.imu = ImuClass(0.0)
        self.gps = GpsClass(0.0)
        self.enc = EncClass(0.0)
        self.ecu = EcuClass(0.0)
        self.est = Estimator(0.0,50.0,0.0,0.0,self.Q,self.R)

    def replay(self,indicator):
        for i in range(len(self.KF_a_his)):
            # READ SENSOR DATA
            self.gps.x       = self.KF_x_his[i]
            self.gps.y       = self.KF_y_his[i]
            self.imu.ax      = self.KF_ax_his[i]
            self.imu.ay      = self.KF_ay_his[i]
            self.imu.psiDot  = self.KF_psiDot_his[i]
            self.enc.v_meas  = self.KF_v_meas_his[i]
            self.ecu.a       = self.KF_a_his[i]
            self.ecu.df      = self.KF_df_his[i]
            if indicator == 1:
                self.est.estimateState(self.imu,self.gps,self.enc,self.ecu,self.est.ekfSwitchVy)
            elif indicator == 2:
                self.est.estimateState(self.imu,self.gps,self.enc,self.ecu,self.est.ekfWithoutVy)
            elif indicator ==3:
                self.est.estimateState(self.imu,self.gps,self.enc,self.ecu,self.est.ekfWithVy)
            self.est.saveHistory()

    def yawPlot(self,ax,indicator):
        if indicator == 1:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.yaw_est_his[self.idx_min:self.idx_max],"o--",label="Switch vy",    alpha=0.7)
        elif indicator == 2:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.yaw_est_his[self.idx_min:self.idx_max],"s--",label="Without vy",   alpha=0.7)
        elif indicator == 3:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.yaw_est_his[self.idx_min:self.idx_max],"v--",label="With vy",      alpha=0.7)
        elif indicator == 0:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.yaw_est_his[self.idx_min:self.idx_max],    "p--",label="exp",          alpha=0.7)
        ax.legend()

    def axPlot(self,ax,indicator):
        if indicator == 1:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.ax_est_his[self.idx_min:self.idx_max],"o--",label="Switch vy",    alpha=0.7)
        elif indicator == 2:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.ax_est_his[self.idx_min:self.idx_max],"s--",label="Without vy",   alpha=0.7)
        elif indicator == 3:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.ax_est_his[self.idx_min:self.idx_max],"v--",label="With vy",      alpha=0.7)
        elif indicator == 0:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.ax_est_his[self.idx_min:self.idx_max],    "p--",label="exp",          alpha=0.7)
        else:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.KF_ax_his[self.idx_min:self.idx_max],     "^--",label="raw",          alpha=0.7)
        ax.legend()

    def ayPlot(self,ax,indicator):
        if indicator == 1:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.ay_est_his[self.idx_min:self.idx_max],"o--",label="Switch vy",    alpha=0.7)
        elif indicator == 2:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.ay_est_his[self.idx_min:self.idx_max],"s--",label="Without vy",   alpha=0.7)
        elif indicator == 3:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.ay_est_his[self.idx_min:self.idx_max],"v--",label="With vy",      alpha=0.7)
        elif indicator == 0:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.ay_est_his[self.idx_min:self.idx_max],    "p--",label="exp",          alpha=0.7)
        else:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.KF_ay_his[self.idx_min:self.idx_max],     "^--",label="raw",          alpha=0.7)
        ax.legend()

    def vxPlot(self,ax,indicator):
        if indicator == 1:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.vx_est_his[self.idx_min:self.idx_max],"o--",label="Switch vy",    alpha=0.7)
        elif indicator == 2:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.vx_est_his[self.idx_min:self.idx_max],"s--",label="Without vy",   alpha=0.7)
        elif indicator == 3:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.vx_est_his[self.idx_min:self.idx_max],"v--",label="With vy",      alpha=0.7)
        elif indicator == 0:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.vx_est_his[self.idx_min:self.idx_max],    "p--",label="exp",          alpha=0.7)
        else:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.KF_v_meas_his[self.idx_min:self.idx_max], "^--",label="raw",          alpha=0.7)
        ax.legend()

    def vyPlot(self,ax,indicator):
        if indicator == 1:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.vy_est_his[self.idx_min:self.idx_max],"o--",label="Switch vy",    alpha=0.7)
        elif indicator == 2:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.vy_est_his[self.idx_min:self.idx_max],"s--",label="Without vy",   alpha=0.7)
        elif indicator == 3:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.vy_est_his[self.idx_min:self.idx_max],"v--",label="With vy",      alpha=0.7)
        elif indicator == 0:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.vy_est_his[self.idx_min:self.idx_max],    "p--",label="exp",          alpha=0.7)
        ax.legend()

    def psiDotPlot(self,ax,indicator):
        if indicator == 1:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.psiDot_est_his[self.idx_min:self.idx_max],"o--",label="Switch vy",    alpha=0.7)
        elif indicator == 2:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.psiDot_est_his[self.idx_min:self.idx_max],"s--",label="Without vy",   alpha=0.7)
        elif indicator == 3:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.est.psiDot_est_his[self.idx_min:self.idx_max],"v--",label="With vy",      alpha=0.7)
        elif indicator == 0:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.psiDot_est_his[self.idx_min:self.idx_max],    "p--",label="exp",          alpha=0.7)
        else:
            ax.plot(self.estimator_time[self.idx_min:self.idx_max],self.KF_psiDot_his[self.idx_min:self.idx_max],     "^--",label="raw",          alpha=0.7)
        ax.legend()

    def trajectoryPlot(self,ax,indicator):
        if indicator == 1:
            ax.plot(self.est.x_est_his[self.idx_min:self.idx_max], self.est.y_est_his[self.idx_min:self.idx_max],     "o--",label="Switch vy",    alpha=0.7)
        elif indicator == 2:
            ax.plot(self.est.x_est_his[self.idx_min:self.idx_max], self.est.y_est_his[self.idx_min:self.idx_max],     "s--",label="Without vy",   alpha=0.7)
        elif indicator == 3:
            ax.plot(self.est.x_est_his[self.idx_min:self.idx_max], self.est.y_est_his[self.idx_min:self.idx_max],     "v--",label="With vy",      alpha=0.7)
        elif indicator == 0:
            ax.plot(self.x_est_his[self.idx_min:self.idx_max],     self.y_est_his[self.idx_min:self.idx_max],         "p--",label="exp",          alpha=0.7)
        else:
            ax.plot(self.KF_x_his[self.idx_min:self.idx_max],self.KF_y_his[self.idx_min:self.idx_max],                "^--",label="raw",          alpha=0.7)
            ax.plot(self.track.nodes[0],        self.track.nodes[1],        color="grey",linestyle="--",    alpha=0.3)
            ax.plot(self.track.nodes_bound1[0], self.track.nodes_bound1[1], color="red",                    alpha=0.3)
            ax.plot(self.track.nodes_bound2[0], self.track.nodes_bound2[1], color="red",                    alpha=0.3)
        ax.axis("equal")
        ax.legend()

    def playBackClean(self):
        self.est.x_est              = 0.0
        self.est.y_est              = 0.0
        self.est.vx_est             = 0.0
        self.est.vy_est             = 0.0
        self.est.v_est              = 0.0
        self.est.ax_est             = 0.0
        self.est.ay_est             = 0.0
        self.est.yaw_est            = 0.0
        self.est.psiDot_est         = 0.0
        self.est.psiDot_drift_est   = 0.0
        self.est.x_est_his          = []
        self.est.y_est_his          = []
        self.est.vx_est_his         = []
        self.est.vy_est_his         = []
        self.est.v_est_his          = []
        self.est.ax_est_his         = []
        self.est.ay_est_his         = []
        self.est.yaw_est_his        = []
        self.est.psiDot_est_his     = []
        self.est.P              = np.eye(np.size(self.est.Q,0))
        self.est.z              = np.zeros(np.size(self.est.Q,0))

class Estimator(object):

    def __init__(self,t0,loop_rate,a_delay,df_delay,Q,R):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        dt          = 1.0 / loop_rate
        self.rate   = 50.0
        L_f         = 0.125
        L_r         = 0.125
        self.vhMdl  = (L_f, L_r)
        self.Q      = Q
        self.R      = R
        self.P      = np.eye(np.size(Q,0)) # initializationtial covariance matrix
        self.z      = np.zeros(np.size(Q,0)) # initial state mean
        self.dt     = dt
        self.a_delay        = a_delay
        self.df_delay       = df_delay
        self.motor_his      = [0.0]*int(a_delay/dt)
        self.servo_his      = [0.0]*int(df_delay/dt)

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

        # SAVE THE measurement/input SEQUENCE USED BY KF
        self.x_his      = [0.0]
        self.y_his      = [0.0]
        self.v_meas_his = [0.0]
        self.ax_his     = [0.0]
        self.ay_his     = [0.0]
        self.psiDot_his = [0.0]
        self.a_his      = [0.0]
        self.df_his     = [0.0]

    # ecu command update
    def estimateState(self,imu,gps,enc,ecu,KF):
        """Do extended Kalman filter to estimate states"""

        self.motor_his.append(ecu.a)
        self.servo_his.append(ecu.df)
        u = [self.motor_his.pop(0), self.servo_his.pop(0)]
        
        # y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot])
        y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot, 0.5*u[1]*enc.v_meas])
        # y = np.array([gps.x_ply, gps.y_ply, enc.v_meas, imu.ax, imu.ay, imu.psiDot])
        KF(y,u)

        # SAVE THE measurement/input SEQUENCE USED BY KF
        self.x_his.append(y[0])
        self.y_his.append(y[1])
        self.v_meas_his.append(y[2])
        self.ax_his.append(y[3])
        self.ay_his.append(y[4])
        self.psiDot_his.append(y[5])
        self.a_his.append(u[0])
        self.df_his.append(u[1])

    def ekfSwitchVy(self, y, u):
        idx = []
        if y[2] > 1.8 or y[5] > 1.0: # vx and psiDot criterion for Vy switch
            idx.append(6)
            self.Q[6,6] = 10.0  # Q_psi
        else:
            self.Q[6,6] = 0.1   # Q_psi

        xDim    = self.z.size                               # dimension of the state
        mx_kp1  = self.f(self.z, u)                         # predict next state
        A       = self.numerical_jac(self.f, self.z, u)     # linearize process model about current state
        P_kp1   = dot(dot(A,self.P),A.T) + self.Q           # proprogate variance
        my_kp1  = self.h(mx_kp1, u)                         # predict future output
        H       = self.numerical_jac(self.h, mx_kp1, u)     # linearize measurement model about predicted next state

        H      = np.delete(H,(idx),axis=0)
        R      = np.delete(self.R,(idx),axis=0)
        R      = np.delete(R,(idx),axis=1)
        y      = np.delete(y,(idx),axis=0)
        my_kp1 = np.delete(my_kp1,(idx),axis=0)
        P12    = dot(P_kp1, H.T)                      # cross covariance
        K      = dot(P12, inv( dot(H,P12) + R))       # Kalman filter gain
        self.z = mx_kp1 + dot(K,(y - my_kp1))
        self.P = dot(dot(K,R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )
        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z

    def ekfWithVy(self, y, u):
        xDim    = self.z.size                           # dimension of the state
        mx_kp1  = self.f(self.z, u)                     # predict next state
        A       = self.numerical_jac(self.f, self.z, u) # linearize process model about current state
        P_kp1   = dot(dot(A,self.P),A.T) + self.Q           # proprogate variance
        my_kp1  = self.h(mx_kp1, u)                              # predict future output
        H       = self.numerical_jac(self.h, mx_kp1, u)     # linearize measurement model about predicted next state
        P12     = dot(P_kp1, H.T)                           # cross covariance
        K       = dot(P12, inv( dot(H,P12) + self.R))       # Kalman filter gain
        
        self.z  = mx_kp1 + dot(K,(y - my_kp1))
        self.P  = dot(dot(K,self.R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )

        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z

    def ekfWithoutVy(self, y, u):
        xDim    = self.z.size                               # dimension of the state
        mx_kp1  = self.f(self.z, u)                         # predict next state
        A       = self.numerical_jac(self.f, self.z, u)     # linearize process model about current state
        P_kp1   = dot(dot(A,self.P),A.T) + self.Q           # proprogate variance
        my_kp1  = self.h(mx_kp1, u)                         # predict future output
        H       = self.numerical_jac(self.h, mx_kp1, u)     # linearize measurement model about predicted next state

        idx = [6]
        H      = np.delete(H,(idx),axis=0)
        R      = np.delete(self.R,(idx),axis=0)
        R      = np.delete(R,(idx),axis=1)
        y      = np.delete(y,(idx),axis=0)
        my_kp1 = np.delete(my_kp1,(idx),axis=0)
        P12    = dot(P_kp1, H.T)                      # cross covariance
        K      = dot(P12, inv( dot(H,P12) + R))       # Kalman filter gain
        self.z = mx_kp1 + dot(K,(y - my_kp1))
        self.P = dot(dot(K,R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )
        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z

    def ekfMultiRate(self, y, u):
        xDim    = self.z.size                               # dimension of the state
        mx_kp1  = self.f(self.z, u)                         # predict next state
        A       = self.numerical_jac(self.f, self.z, u)     # linearize process model about current state
        P_kp1   = dot(dot(A,self.P),A.T) + self.Q           # proprogate variance
        my_kp1  = self.h(mx_kp1, u)                         # predict future output
        H       = self.numerical_jac(self.h, mx_kp1, u)     # linearize measurement model about predicted next state

        idx = []
        if self.x_his[-1] == y[0]:
            idx.append(0)
        if self.y_his[-1] == y[1]:
            idx.append(1)
        if self.v_meas_his[-1] == y[2]:
            idx.append(2)
        if self.ax_his[-1] == y[3]:
            idx.append(3)
        if self.ay_his[-1] == y[4]:
            idx.append(4)
        if self.psiDot_his[-1] == y[5]:
            idx.append(5)

        if len(idx) == 6:
            print "No measurement data received!"
            (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = mx_kp1
        else:
            H      = np.delete(H,(idx),axis=0)
            R      = np.delete(self.R,(idx),axis=0)
            R      = np.delete(R,(idx),axis=1)
            y      = np.delete(y,(idx),axis=0)
            my_kp1 = np.delete(my_kp1,(idx),axis=0)
            P12    = dot(P_kp1, H.T)                      # cross covariance
            K      = dot(P12, inv( dot(H,P12) + R))       # Kalman filter gain
            self.z = mx_kp1 + dot(K,(y - my_kp1))
            self.P = dot(dot(K,R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T )
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

    def h(self, x, u):
        """ This is the measurement model to the kinematic<->sensor model above """
        y = [0]*7
        y[0] = x[0]      # x
        y[1] = x[1]      # y
        y[2] = x[2]      # vx
        y[3] = x[4]      # a_x
        y[4] = x[5]      # a_y
        y[5] = x[7]      # psiDot
        y[6] = x[3]      # vy
        return np.array(y)

    def saveHistory(self):
        # self.time_his.append(self.curr_time)

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
    def __init__(self,t0):
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

class GpsClass(object):
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        # GPS measurement
        self.x      = 0.0
        self.y      = 0.0
        self.x_ply  = 0.0
        self.y_ply  = 0.0
        
        # GPS measurement history
        self.x_his      = np.array([])
        self.y_his      = np.array([])
        self.x_ply_his  = np.array([0.0])
        self.y_ply_his  = np.array([0.0])


class EncClass(object):
    
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """

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

class EcuClass(object):

    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """

        # ECU measurement
        self.a  = 0.0
        self.df = 0.0
        
        # ECU measurement history
        self.a_his  = []
        self.df_his = []
        
        # time stamp
        self.t0         = t0
        self.time_his   = []

if __name__ == '__main__':
    main()