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

from numpy import sin, cos, arctan, array, dot

# discrete non-linear bicycle model dynamics
# function wrapper, this function returns a function
def f_2s_disc(z, u, vhMdl, trMdl, dt, v_x): 
    """
    process model
    input: state z at time k, z[k] := [beta[k], r[k]], (i.e. slip angle and yaw rate)
    output: state at next time step (k+1)
    """
    
    # get states / inputs
    beta    = z[0]
    r       = z[1]
    d_f     = u
    
    # extract parameters
    (a,b,m,I_z) = vhMdl
    (trMdlFront, trMdlRear) = trMdl

    # comptue the front/rear slip  [rad/s]
    # ref: Hindiyeh Thesis, p58
    a_F     = arctan(beta + a*r/v_x) - d_f
    a_R     = arctan(beta - b*r/v_x)

    # compute tire force
    FyF     = f_pajecka(trMdlFront, a_F)
    FyR     = f_pajecka(trMdlRear, a_R)

    # compute next state
    beta_next   = beta  + dt*(-r + (1/(m*v_x))*(FyF*cos(d_f)+FyR))
    r_next      = r    + dt/I_z*(a*FyF*cos(d_f) - b*FyR);
    return array([beta_next, r_next])

C = array([[0, 1]])
def h_2s_disc(x):
    """
    measurement model
    state: z := [beta, r], (i.e. slip angle and yaw rate)
    output h := r (yaw rate)
    """
    return dot(C, x)
    
def f_pajecka(trMdl, alpha):
    """
    f_pajecka = d*sin(c*atan(b*alpha))    
    
    inputs :
        * trMdl := tire model, a list or tuple of parameters (b,c,d)
        * alpha := tire slip angle [radians]
    outputs :
        * Fy := lateral force from tire [Newtons]
    """
    (b,c,d) = trMdl
    return  d*sin(c*arctan(b*alpha)) 
