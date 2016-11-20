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

from numpy import sin, cos, tan, arctan, array, dot
from numpy import sign, argmin, sqrt, abs
import rospy

# discrete non-linear bicycle model dynamics 6-dof
def f_6s(z, u, vhMdl, trMdl, F_ext, dt): 
    """
    process model
    input: state z at time k, z[k] := [X[k], Y[k], phi[k], v_x[k], v_y[k], r[k]])
    output: state at next time step z[k+1]
    """
    
    # get states / inputs
    X       = z[0]
    Y       = z[1]
    phi     = z[2]
    v_x     = z[3]
    v_y     = z[4]
    r       = z[5]

    d_f     = u[0]
    FxR     = u[1]

    # extract parameters
    (a,b,m,I_z)             = vhMdl
    (a0, Ff)                = F_ext
    (trMdlFront, trMdlRear) = trMdl
    (B,C,mu)                = trMdlFront
    g                       = 9.81
    Fn                      = m*g/2.0         # assuming a = b (i.e. distance from CoG to either axel)

    # limit force to tire friction circle
    if FxR >= mu*Fn:
        FxR = mu*Fn

    # comptue the front/rear slip  [rad/s]
    # ref: Hindiyeh Thesis, p58
    a_F     = arctan((v_y + a*r)/v_x) - d_f
    a_R     = arctan((v_y - b*r)/v_x)

    # compute lateral tire force at the front
    TM_param    = [B, C, mu*Fn]
    FyF         = -f_pajecka(TM_param, a_F)

    # compute lateral tire force at the rear
    # ensure that magnitude of longitudinal/lateral force lie within friction circle
    FyR_paj     = -f_pajecka(TM_param, a_R)
    FyR_max     = sqrt((mu*Fn)**2 - FxR**2)
    Fy          = array([FyR_max, FyR_paj])
    idx         = argmin(abs(Fy))
    FyR         = Fy[idx]

    # compute next state
    X_next      = X + dt*(v_x*cos(phi) - v_y*sin(phi)) 
    Y_next      = Y + dt*(v_x*sin(phi) + v_y*cos(phi)) 
    phi_next    = phi + dt*r
    v_x_next    = v_x + dt*(r*v_y +1/m*(FxR - FyF*sin(d_f)) - (a0*v_x**2 + Ff)*sign(v_x) )
    v_y_next    = v_y + dt*(-r*v_x +1/m*(FyF*cos(d_f) + FyR))
    r_next      = r    + dt/I_z*(a*FyF*cos(d_f) - b*FyR)

    return array([X_next, Y_next, phi_next, v_x_next, v_y_next, r_next])
    
   
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


def f_KinBkMdl(z,u,vhMdl, F_ext, dt):
    """
    process model
    input: state z at time k, z[k] := [x[k], y[k], psi[k], v[k]]
    output: state at next time step z[k+1]
    """
    
    # get states / inputs
    x       = z[0]
    y       = z[1]
    psi     = z[2]
    v       = z[3]

    d_f     = u[0]
    a       = u[1]

    # extract parameters
    (L_a, L_b)             = vhMdl
    (a0, Ff)               = F_ext

    # compute slip angle
    bta         = arctan( L_a / (L_a + L_b) * tan(d_f) )

    # compute next state
    x_next      = x + dt*( v*cos(psi + bta) ) 
    y_next      = y + dt*( v*sin(psi + bta) ) 
    psi_next    = psi + dt*v/L_b*sin(bta)
    v_next      = v + dt*(a  - (a0*v**2 + Ff)*sign(v) )

    if abs(v_next) < 0.0005:
        v_next = 0

    return array([x_next, y_next, psi_next, v_next])
 
