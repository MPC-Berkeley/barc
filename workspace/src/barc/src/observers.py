#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu)  Development of the web-server app Dator was 
# based on an open source project by Bruce Wootton, with contributions from 
# Kiet Lam (kiet.lam@berkeley.edu)   
# ---------------------------------------------------------------------------

from numpy import array, dot, eye

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
