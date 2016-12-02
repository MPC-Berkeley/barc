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

def f_SensorKinematicModel(z, u, vhMdl, dt, est_mode):
    """ This Sensor model contains a pure Sensor-Model and a Kinematic model. They're independent from each other."""
    (l_A,l_B) = vhMdl
    bta = math.atan2(l_A*tan(u[1]),l_A+l_B)
    zNext = [0]*14
    zNext[0] = z[0] + dt*(cos(z[6])*z[2] - sin(z[6])*z[3])          # x
    zNext[1] = z[1] + dt*(sin(z[6])*z[2] + cos(z[6])*z[3])          # y
    zNext[2] = z[2] + dt*(z[4]+z[7]*z[3])                           # v_x
    zNext[3] = z[3] + dt*(z[5]-z[7]*z[2])                           # v_y
    zNext[4] = z[4]                                                 # a_x
    zNext[5] = z[5]                                                 # a_y
    zNext[6] = z[6] + dt*(z[7])                                     # psi
    zNext[7] = z[7]                                                 # psidot
    zNext[8] = z[8]                                                 # drift_psi
    zNext[9] = z[9] + dt*(z[12]*cos(z[11] + bta))                   # x
    zNext[10] = z[10] + dt*(z[12]*sin(z[11] + bta))                 # y
    zNext[11] = z[11] + dt*(z[12]/l_B*sin(bta))                     # psi
    zNext[12] = z[12] + dt*(u[0] - 0.5*z[12])                       # v
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
