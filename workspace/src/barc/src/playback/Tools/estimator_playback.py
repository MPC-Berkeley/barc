import os
import sys
homedir = os.path.expanduser("~")
sys.path.append(os.path.join(homedir,"barc/workspace/src/barc/src"))
# from Localization_helpers import Track
from barc.msg import ECU, pos_info, Vel_est
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_imu_fusion
from std_msgs.msg import Header
from numpy import eye, zeros, diag, tan, cos, sin, vstack, linalg, pi
from numpy import ones, polyval, size, dot, add
from scipy.linalg import inv, cholesky
from tf import transformations
import math
import numpy as np
import pdb

def approximate_yaw(x, y, time, time_now):
    index = np.argmin(abs(time - time_now))

    x_0 = x[index - 1]
    y_0 = y[index - 1]

    x_1 = x[index]
    y_1 = y[index]

    argument_y = y_1 - y_0
    argument_x = x_1 - x_0
    # angle = np.arctan(argument_y / argument_x)

    angle = 0.0

    if argument_x > 0.0:
        angle = np.arctan(argument_y / argument_x)

    if argument_y >= 0.0 and argument_x < 0.0:
        angle = np.pi + np.arctan(argument_y / argument_x)

    if argument_y < 0.0 and argument_x < 0.0:
        angle = - np.pi + np.arctan(argument_y / argument_x)

    if argument_y > 0.0 and argument_x == 0.0:
        angle = np.pi / 2.0

    if argument_y < 0.0 and argument_x == 0.0:
        angle = - np.pi / 2.0

    if angle < 0.0:
        angle += 2.0 * np.pi

    # angle = np.arctan2(argument_y, argument_x)

    return angle

def main():
    # node initialization
    a_delay     = 0.2
    df_delay    = 0.0
    loop_rate   = 50.0
   
    Q = eye(8)
    Q[0,0] = 0.01 # x
    Q[1,1] = 0.01 # y
    Q[2,2] = 0.01 # vx
    Q[3,3] = 0.01 # vy
    Q[4,4] = 1.0 # ax
    Q[5,5] = 1.0 # ay
    Q[6,6] = 0.0001 # psi
    Q[7,7] = 0.1 # 1.0 # psidot
    R = eye(6)
    R[0,0] = 1.0    # x
    R[1,1] = 1.0    # y
    R[2,2] = 0.1    # vx
    R[3,3] = 10.0   # ax
    R[4,4] = 100.0   # ay
    R[5,5] = 0.001  # psiDot

    imu = ImuClass(0.0)
    gps = GpsClass(0.0)
    enc = EncClass(0.0)
    ecu = EcuClass(0.0)
    est = Estimator(0.0,loop_rate,a_delay,df_delay,Q,R)
    
    # track = Track(0.01,1.0)
    # track.createRaceTrack()

    estMsg = pos_info()
    
    # homedir = os.path.expanduser("~")
    homedir = "/home/lukas/barc/workspace/src/barc/src/playback/1_two_beacons_pf/"

    pathSave = os.path.join(homedir,"estimator_output.npz")
    npz_output = np.load(pathSave)
    KF_x_his            = npz_output["KF_x_his"]
    KF_y_his            = npz_output["KF_y_his"]
    KF_v_meas_his       = npz_output["KF_v_meas_his"]
    KF_ax_his           = npz_output["KF_ax_his"]
    KF_ay_his           = npz_output["KF_ay_his"]
    KF_psiDot_his       = npz_output["KF_psiDot_his"]
    KF_a_his            = npz_output["KF_a_his"]
    KF_df_his           = npz_output["KF_df_his"]
    estimator_time      = npz_output["estimator_time"]
    print "Finish loading data from", pathSave

    pathSave = os.path.join(homedir,"estimator_imu.npz")
    npz_imu = np.load(pathSave)
    psiDot_his      = npz_imu["psiDot_his"]
    roll_his        = npz_imu["roll_his"]
    pitch_his       = npz_imu["pitch_his"]
    yaw_his         = npz_imu["yaw_his"]
    ax_his          = npz_imu["ax_his"]
    ay_his          = npz_imu["ay_his"]
    imu_time        = npz_imu["imu_time"]
    print "Finish loading data from", pathSave

    pathSave = os.path.join(homedir,"estimator_gps.npz")
    npz_gps = np.load(pathSave)
    x_his       = npz_gps["x_his"]
    y_his       = npz_gps["y_his"]
    x_ply_his   = npz_gps["x_ply_his"]
    y_ply_his   = npz_gps["y_ply_his"]
    gps_time    = npz_gps["gps_time"]
    gps_angle   = npz_gps["angle_his"]
    gps_ply_time= npz_gps["gps_ply_time"]
    print "Finish loading data from", pathSave

    pathSave = os.path.join(homedir,"estimator_enc.npz")
    npz_enc = np.load(pathSave)
    v_fl_his    = npz_enc["v_fl_his"]
    v_fr_his    = npz_enc["v_fr_his"]
    v_rl_his    = npz_enc["v_rl_his"]
    v_rr_his    = npz_enc["v_rr_his"]
    v_meas_his  = npz_enc["v_meas_his"]
    enc_time    = npz_enc["enc_time"]
    print "Finish loading data from", pathSave

    pathSave = os.path.join(homedir,"estimator_ecu.npz")
    npz_ecu = np.load(pathSave)
    a_his       = npz_ecu["a_his"]
    df_his      = npz_ecu["df_his"]
    ecu_time    = npz_ecu["ecu_time"]
    print "Finish loading data from", pathSave

    for i in range(len(KF_a_his)):
        # READ SENSOR DATA
        gps.x       = KF_x_his[i]
        gps.y       = KF_y_his[i]
        imu.ax      = KF_ax_his[i]
        imu.ay      = KF_ay_his[i]
        imu.psiDot  = KF_psiDot_his[i]
        enc.v_meas  = KF_v_meas_his[i]
        ecu.a       = KF_a_his[i]
        ecu.df      = KF_df_his[i]

        est.estimateState(imu,gps,enc,ecu,est.ekf)
        est.saveHistory()

    gps_t, indices = np.unique(gps_time, return_index=True)
    gps_yaw = zeros(len(x_his[indices])-1)
    for i in range(1,len(x_his[indices])):
        if gps_t[i] > enc_time[np.argmax(v_meas_his>0.1)] :
            gps_yaw[i-1] = approximate_yaw(x_his[indices], y_his[indices], gps_t, gps_t[i])
    gps_yaw = np.unwrap(gps_yaw)

    gps_angle = gps_angle * np.pi / 180.0 - np.pi / 2.0

    homedir = os.path.expanduser("~")
    pathSave = os.path.join(homedir,"barc_debugging2/estimator_output.npz")
    np.savez(pathSave,yaw_est_his       = est.yaw_est_his,
                      psiDot_est_his    = est.psiDot_est_his,
                      x_est_his         = est.x_est_his,
                      y_est_his         = est.y_est_his,
                      vx_est_his        = est.vx_est_his,
                      vy_est_his        = est.vy_est_his,
                      ax_est_his        = est.ax_est_his,
                      ay_est_his        = est.ay_est_his,
                      KF_x_his          = KF_x_his,
                      KF_y_his          = KF_y_his,
                      KF_v_meas_his     = KF_v_meas_his,
                      KF_ax_his         = KF_ax_his,
                      KF_ay_his         = KF_ay_his,
                      KF_psiDot_his     = KF_psiDot_his,
                      KF_a_his          = KF_a_his,
                      KF_df_his         = KF_df_his,
                      estimator_time    = estimator_time)

    pathSave = os.path.join(homedir,"barc_debugging2/estimator_imu.npz")
    np.savez(pathSave,psiDot_his    = psiDot_his,
                      roll_his      = roll_his,
                      pitch_his     = pitch_his,
                      yaw_his       = yaw_his,
                      ax_his        = ax_his,
                      ay_his        = ay_his,
                      imu_time      = imu_time)

    pathSave = os.path.join(homedir,"barc_debugging2/estimator_gps.npz")
    np.savez(pathSave,x_his         = x_his,
                      y_his         = y_his,
                      x_ply_his     = x_ply_his,
                      y_ply_his     = y_ply_his,
                      gps_t         = gps_t,
                      gps_angle     = gps_angle,
                      gps_yaw       = gps_yaw,
                      gps_time      = gps_time,
                      gps_ply_time  = gps_ply_time)

    pathSave = os.path.join(homedir,"barc_debugging2/estimator_enc.npz")
    np.savez(pathSave,v_fl_his          = v_fl_his,
                      v_fr_his          = v_fr_his,
                      v_rl_his          = v_rl_his,
                      v_rr_his          = v_rr_his,
                      v_meas_his        = v_meas_his,
                      enc_time          = enc_time)

    pathSave = os.path.join(homedir,"barc_debugging2/estimator_ecu.npz")
    np.savez(pathSave,a_his         = a_his,
                      df_his        = df_his,
                      ecu_time      = ecu_time)

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

        self.a_his.append(ecu.a)
        self.df_his.append(ecu.df)
        u = [self.a_his.pop(0), self.df_his.pop(0)]
        
        y = np.array([gps.x, gps.y, enc.v_meas, imu.ax, imu.ay, imu.psiDot])
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

    def ekf(self, y, u):
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

    def ekfMultiRate(self, y, u):
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
        y = [0]*6
        y[0] = x[0]      # x
        y[1] = x[1]      # y
        y[2] = x[2]      # vx
        y[3] = x[4]      # a_x
        y[4] = x[5]      # a_y
        y[5] = x[7]      # psiDot
        # y[6] = x[3]      # vy
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
