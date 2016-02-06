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

from numpy import array, dot, cos, sin

def estimateAngularAcceleration(imu_data, w_z_prev, dt):
    # compute dot_w_z
    (_, _, _, w_z_new)    = imu_data.getFilteredSignal()
    dwz = (w_z_new - w_z_prev)/dt

    return (dwz, w_z_new)

       
# estimate position in the global frame
def estimatePosition( v_BF, X_GF, psi, dt):
    """
    function    : estimate_position
    
    input       : current velocity estimates in the body frame, and 
                  position estimates in the global frame
    output      : current state estimate for (v_x, v_y) in local frame and 
                  (X,Y,v_X, v_Y) in global frame
    
    assumptions: a_z = v_z = w_x = w_y = 0
    """
    
    #### unpack signal data
    (v_x , v_y)             = v_BF.getRawSignal()    
    X_t                     = X_GF.getFilteredSignal()
    
    # compute the position    
    A = array([[1,      0,      dt,     0],
               [0,      1,      0,      dt],	
               [0,      0,      0,      0],
               [0,      0,      0,      0],])
    B = array([[0,0],
               [0,0],
               [1,0],
               [0,1]])
    R = array([[cos(psi),   sin(psi)],
               [-sin(psi),  cos(psi)]])
    u = dot(R, array([v_x,v_y]))
    X_next = dot(A, X_t) + dot(B,u)
    X_GF.update(list(X_next))
