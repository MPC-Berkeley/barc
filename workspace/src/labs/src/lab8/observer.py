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
from numpy import sqrt
from numpy import pi, cos, sin, tan, arctan
from numpy import eye, array, zeros, unwrap, diag, copy, dot
from scipy.linalg import inv
from tf import transformations


class EncoderModel:
    def __init__(self):
        # encoder measurement variables
        self.vhat_m1        = 0
        self.vhat_m2        = 0
        self.t0_m1          = 0 
        self.t0_m2          = 0 
        self.r_tire         = 0.05 # radius of the tire 
        self.ang_km1        = 0 
        self.ang_km2        = 0
        self.s_m1           = 0         # distance travelled
        self.s_m2           = 0         # distance travelled

    # This call takes the encoder tick counts as input and returns a filtered velocity estimate
    # M1 = method 1
    def estimateVelocityM1(self,data):
        n_FL = data.FL
        n_FR = data.FR
        n_BL = data.BL
        n_BR = data.BR

        # compute the average encoder measurement
        n_mean = (n_FL + n_FR)/2

        # transfer the encoder measurement to angular displacement
        ang_mean = n_mean*2*pi/8

        # compute time elapsed
        tf = time.time()
        dt = tf - self.t0_m1
        
        # compute speed with second-order, backwards-finite-difference estimate
        # compute distance
        self.vhat_m1    = self.r_tire*(ang_mean - 4*self.ang_km1 + 3*self.ang_km2)/(dt)
        self.s_m1       += self.vhat_m1*dt

        # update
        self.ang_km2 = self.ang_km1
        self.ang_km1 = ang_mean
        self.t0_m1   = time.time()

    # This callback takes velocity estimates directly as input from the arduino and filters them
    # M2 = method 2
    def estimateVelocityM2(self,data):
        # compute time elapsed
        tf      = time.time()
        dt      = tf - self.t0_m2

        # get velocity estimates per wheel
        v_FL    = data.FL
        v_FR    = data.FR
        v_BL    = data.BL
        v_BR    = data.BR
        
        # compute speed and distance travelled
        self.vhat_m2    = (v_BL + v_BR)/2
        self.s_m2       += self.vhat_m2 * dt
        self.t0_m2      = time.time()


class ImuModel:
    def __init__(self):
        # units: [rad] and [rad/s]
        # orientation (roll, pitch, yaw), angular velocity, linear acceleration
        self.r          = 0
        self.p          = 0
        self.y          = 0
        self.wx         = 0
        self.wy         = 0
        self.wz         = 0
        self.ax         = 0
        self.ay         = 0
        self.az         = 0

        # planar acceleration 
        self.a          = 0

        # history
        self.y0         = None
        self.y_prev     = 0

        # amount of angle deviation from first yaw measurement
        self.dy         = 0.0
        self.dy_deg     = 0.0
        
    def updateEstimates(self, data):

        # get orientation from quaternion data, and convert to roll, pitch, yaw
        # extract angular velocity and linear acceleration data
        ori         = data.orientation
        quaternion  = (ori.x, ori.y, ori.z, ori.w)
        (r, p, y)   = transformations.euler_from_quaternion(quaternion)

        # save initial measurements
        if self.y0 == None:
            self.y0  = y
    
        # unwrap measurement
        self.y          = unwrap(array([self.y_prev, y]), discont = pi)[1]
        self.y_prev     = self.y
        self.dy         = self.y - self.y0
        self.dy_deg     = self.dy*(180.0/pi)
    
        # save orientation, angular velocity, linear acceleration data
        self.r      = r
        self.p      = p
        self.wx     = data.angular_velocity.x
        self.wy     = data.angular_velocity.y
        self.wz     = data.angular_velocity.z
        self.ax     = data.linear_acceleration.x
        self.ay     = data.linear_acceleration.y
        self.az     = data.linear_acceleration.z

        self.a      = sqrt( self.ax**2 + self.ay**2 )


class GPS:
    def __init__(self, dt=0.1):
        self.x          = 0
        self.y          = 0
        self.t          = 0
        self.x_prev     = None
        self.y_prev     = None
        self.t_prev     = None
        self.vx         = None
        self.vy         = None

    def updateEstimates(self, data):
        self.t = data.timestamp_ms / 1000.0 
        self.x = data.x_m
        self.y = data.y_m

        if self.t_prev != None:
            self.vx     = (self.x - self.x_prev) / (self.t - self.t_prev)
            self.vy     = (self.y - self.y_prev) / (self.t - self.t_prev)

        self.x_prev = self.x
        self.y_prev = self.y
        self.t_prev = self.t


# state estimation node
class Observer():
    def __init__(self, dt=0.1):
        # state vector
        self.zhat   = array([0.0, 0.0, 0.0, 0.0])       # z = [x,y,psi,v]
        self.xDim   = self.zhat.size

        # input vector
        self.u      = array([0.0, 0.0])                 # u = [df, a]

        # model parameters
        self.La     = 0.16 # meters          
        self.Lb     = 0.16 # meters     
        self.dt     = dt

        # sensors models
        self.imu    = ImuModel()
        self.enc    = EncoderModel()
        self.y      = array([0.0, 0.0])                 # y = [psi, v]

        # ekf parameters
        self.P      = eye(self.xDim) 
        self.Q      = diag([0.1,0.1,0.01,0.5])             
        self.R      = diag([0.01,0.1]) 


    def imuCallback(self, data):
        self.imu.updateEstimates(data) 
        self.y[0]       = self.imu.dy
        self.u[1]       = self.imu.a

    def encCallback(self, data):
        self.enc.estimateVelocityM1(data)
        self.y[1]       = self.enc.vhat_m1

    def controllerCallback(self, data):
        # get steering angle using steering map
        df_pwm = data.servo
        self.u[0] = -0.0012*df_pwm + 1.8962 

    def getStateEstimate(self):
        self.ekf()
        return self.zhat
     
    def f_KinBkMdl(self, z, u):
        # get states / inputs / parameters
        x       = z[0]
        y       = z[1]
        psi     = z[2]
        v       = z[3]
        df      = u[0]
        a       = u[1]
        La      = self.La
        Lb      = self.Lb
        dt      = self.dt

        # compute slip angle
        bta         = arctan( La / (La + Lb) * tan(df) )

        # compute prediction
        x_next      = x     + dt*( v*cos(psi + bta) ) 
        y_next      = y     + dt*( v*sin(psi + bta) ) 
        psi_next    = psi   + dt*v/Lb*sin(bta)
        v_next      = v     + dt*a
        
        return array([x_next, y_next, psi_next, v_next])


    def h_KinBkMdl(self, z, u):
        C = array([[0,0,1,0],[0,0,0,1]])
        return dot(C, z)

    def ekf(self):
        """
         EKF   Extended Kalman Filter for nonlinear dynamic systems
         ekf(f,mx,P,h,z,Q,R) returns state estimate, x and state covariance, P 
         for nonlinear dynamic system:
                   x_k+1 = f(x_k) + w_k
                   y_k   = h(x_k) + v_k
         where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
               v ~ N(0,R) meaning v is gaussian noise with covariance R
        Inputs:    f: function handle for f(x)
                   mx_k: "a priori" state estimate
                   P_k: "a priori" estimated state covariance
                   h: fanction handle for h(x)
                   y_kp1: current measurement
                   Q: process noise covariance 
                   R: measurement noise covariance
                   args: additional arguments to f(x, *args)
        Output:    mx_kp1: "a posteriori" state estimate
                   P_kp1: "a posteriori" state covariance
                   
        Notation: mx_k = E[x_k] and my_k = E[y_k], where m stands for "mean of"
        """
        f       = self.f_KinBkMdl
        h       = self.h_KinBkMdl
        mx_k    = self.zhat 
        u_k     = self.u
        y_kp1   = self.y
        P_k     = self.P
        Q       = self.Q
        R       = self.R

        xDim    = mx_k.size                             # dimension of the state
        mx_kp1  = f(mx_k, u_k)                          # predict next state
        A       = self.numerical_jac(f, mx_k, u_k)      # linearize process model about current state
        P_kp1   = dot(dot(A,P_k),A.T) + Q               # proprogate variance
        my_kp1  = h(mx_kp1, u_k)                        # predict future output
        H       = self.numerical_jac(h, mx_kp1, u_k)    # linearize measurement model about predicted next state
        P12     = dot(P_kp1, H.T)                       # cross covariance
        K       = dot(P12, inv( dot(H,P12) + R))        # Kalman filter gain
        mx_kp1  = mx_kp1 + dot(K,(y_kp1 - my_kp1))      # state estimate
        P_kp1   = dot(dot(K,R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T ) 

        self.zhat   = mx_kp1
        self.P      = P_kp1

    def numerical_jac(self, f, x, u):
        # numerical gradient and diagonal hessian
        y = f(x,u)
        
        jac = zeros( (y.size,x.size) )
        eps = 1e-5
        xp = copy(x)
        
        for i in range(x.size):
            xp[i] = x[i] + eps/2.0
            yhi = f(xp, u)
            xp[i] = x[i] - eps/2.0
            ylo = f(xp, u)
            xp[i] = x[i]
            jac[:,i] = (yhi - ylo) / eps
        return jac

