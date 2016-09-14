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

from numpy import array, dot, eye, copy
from numpy import dot, zeros
from scipy.linalg import inv
import rospy

C = array([[1, 0]])
B = eye(2)
def kinematicLuembergerObserver(vhat_x, vhat_y, w_z, a_x, a_y, v_x_enc, aph, dt):
    """
    inputs: 
        * current longitudinal velocity estimate vhat_x [m/s]
        * current lateral velocity estimate vhat_y [m/s]
        * yaw rate measurement from gyroscope [rad/s]
        * a_x measurement from IMU [m/s^2]
        * a_y measurement from IMU [m/s^2]
        * v_x estimate from encoder (v_x_enc)  [m/s]
    output:
        * longitudinal velocity estimate vhat_x at next time step [m/s]
        * lateral velocity estimate vhat_y at next time step [m/s]

    reference:
        Farrelly, Jim, and Peter Wellstead. "Estimation of vehicle lateral velocity."
        Control Applications, 1996. Proceedings of the 1996 IEEE International Conference
        on IEEE, 1996
        Equations (25) - (31)
    """

    # compute the observer gain
    # note, the reshape(-1,1) transforms the array into a n x 1 vector
    K = array([ 2*aph*abs(w_z), (aph**2 - 1)*w_z ]).reshape(-1,1)

    # if car is not moving, then acclerations should be zero 
    if v_x_enc == 0:
        a_x = 0
        a_y = 0
        vhat_x = 0
        vhat_y = 0

    # build system matrices
    A = array([[ 0,     w_z], \
               [-w_z,   0  ]])
    u = array([a_x, a_y]).reshape(-1,1)
    vhat_k = array([vhat_x, vhat_y]).reshape(-1,1)

    # apply observer
    vhat_kp1 = vhat_k + dt*( dot( (A - dot(K,C)), vhat_k) + dot(B,u) + K*v_x_enc)

    return vhat_kp1


def ekf(f, mx_k, P_k, h, y_kp1, Q, R, args):
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


    xDim    = mx_k.size                         # dimension of the state
    mx_kp1  = f(mx_k, *args)                    # predict next state
    A       = numerical_jac(f, mx_k, *args)     # linearize process model about current state
    P_kp1   = dot(dot(A,P_k),A.T) + Q           # proprogate variance
    my_kp1  = h(mx_kp1)                         # predict future output
    H       = numerical_jac(h, mx_kp1)          # linearize measurement model about predicted next state
    P12     = dot(P_kp1, H.T)                   # cross covariance
    K       = dot(P12, inv( dot(H,P12) + R))    # Kalman filter gain
    mx_kp1  = mx_kp1 + dot(K,(y_kp1 - my_kp1))  # state estimate
    P_kp1   = dot(dot(K,R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T ) 

    return (mx_kp1, P_kp1)


    
def numerical_jac(f,x, *args):
    """
    Function to compute the numerical jacobian of a vector valued function 
    using final differences
    """
    # numerical gradient and diagonal hessian
    y = f(x, *args)
    
    jac = zeros( (y.size,x.size) )
    eps = 1e-5
    xp = copy(x)
    
    for i in range(x.size):
        xp[i] = x[i] + eps/2.0
        yhi = f(xp, *args)
        xp[i] = x[i] - eps/2.0
        ylo = f(xp, *args)
        xp[i] = x[i]
        jac[:,i] = (yhi - ylo) / eps
    return jac
