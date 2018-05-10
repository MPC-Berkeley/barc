#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
from numpy import sign, sqrt
import math

def f_KinBkMdl(z, u, vhMdl, dt, est_mode):
    """
    process model
    input: state z at time k, z[k] := [x[k], y[k], psi[k], v[k]]
    output: state at next time step z[k+1]
    Does not account for drift in psi estimation!
    -> Either put low trust on measured psi-values or add extra state for drift estimation!
    """
    #c = array([0.5431, 1.2767, 2.1516, -2.4169])

    # get states / inputs
    x       = z[0]
    y       = z[1]
    psi     = z[2]
    v       = z[3]

    d_f     = u[0]
    a       = u[1]

    # extract parameters
    (L_a, L_b)             = vhMdl

    # compute slip angle
    bta         = arctan( L_a / (L_a + L_b) * tan(d_f) )

    # compute next state
    x_next      = x + dt*( v*cos(psi + bta) )
    y_next      = y + dt*( v*sin(psi + bta) )
    psi_next    = psi + dt*v/L_b*sin(bta)
    v_next      = v + dt*(a - 0.63*sign(v)*v**2)

    return array([x_next, y_next, psi_next, v_next])

def h_KinBkMdl(x, u, vhMdl, dt, est_mode):
    """
    Measurement model
    """
    if est_mode==1:                     # GPS, IMU, Enc
        C = array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    elif est_mode==2:                     # IMU, Enc
        C = array([[0, 0, 1, 0],
                   [0, 0, 0, 1]])
    elif est_mode==3:                     # GPS
        C = array([[1, 0, 0, 0],
                   [0, 1, 0, 0]])
    elif est_mode==4:                     # GPS, Enc
        C = array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 0, 1]])
    else:
        print("Wrong est_mode")
    return dot(C, x)

def f_SensorKinematicModel(z, u, vhMdl, dt, est_mode):
    """ This Sensor model contains a pure Sensor-Model and a Kinematic model. They're independent from each other."""
    (l_A,l_B) = vhMdl
    bta = math.atan2(l_A*tan(u[1]),l_A+l_B)
    zNext = [0]*14
    zNext[0] = z[0] + dt*(cos(z[6])*z[2] - sin(z[6])*z[3])          # x
    zNext[1] = z[1] + dt*(sin(z[6])*z[2] + cos(z[6])*z[3])          # y
    zNext[2] = z[2] + dt*(z[4]+z[7]*z[3])                           # v_x
    zNext[3] = z[3] + dt*(z[5]-z[7]*z[2])                           # v_y
    # print((z[5]))
    # print((z[5]-z[7]*z[2]))
    zNext[4] = z[4]                                                 # a_x
    zNext[5] = z[5]                                                 # a_y
    zNext[6] = z[6] + dt*(z[7])                                     # psi
    zNext[7] = z[7]                                                 # psidot
    zNext[8] = z[8]                                                 # drift_psi
    zNext[9] = z[9] + dt*(z[12]*cos(z[11] + bta))                   # x
    zNext[10] = z[10] + dt*(z[12]*sin(z[11] + bta))                 # y
    zNext[11] = z[11] + dt*(z[12]/l_B*sin(bta))                     # psi
    zNext[12] = z[12] + dt*(u[0] - 0.05*z[12])                       # v
    zNext[13] = z[13]                                               # drift_psi_2
    return array(zNext)

def h_SensorKinematicModel(x, u, vhMdl, dt, est_mode):
    """ This is the measurement model to the kinematic<->sensor model above """
    y = [0]*13
    y[0] = x[0]                     # x
    y[1] = x[1]                     # y
    y[2] = sqrt(x[2]**2+x[3]**2)    # v
    y[3] = x[6]+x[8]                # psi
    y[4] = x[7]                     # psiDot
    y[5] = x[4]                     # a_x
    y[6] = x[5]                     # a_y
    y[7] = x[9]                     # x
    y[8] = x[10]                    # y
    y[9] = x[11]+x[13]              # psi
    y[10] = x[12]                   # v
    y[11] = x[2]                    # v_x
    y[12] = x[3]                    # v_y
    return array(y)
