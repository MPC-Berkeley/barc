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
    # print "u in sys model", u
    zNext = [0]*8
    zNext[0] = z[0] + dt*(cos(z[6])*z[2] - sin(z[6])*z[3])  # x
    zNext[1] = z[1] + dt*(sin(z[6])*z[2] + cos(z[6])*z[3])  # y
    zNext[2] = z[2] + dt*(z[4]+z[7]*z[3])                   # v_x
    zNext[3] = z[3] + dt*(z[5]-z[7]*z[2])                   # v_y
    zNext[4] = z[4]                                         # a_x
    zNext[5] = z[5]                                         # a_y
    zNext[6] = z[6] + dt*(z[7])                             # psi
    zNext[7] = z[7]                                         # psidot
    return array(zNext)

def h_SensorKinematicModel(x, u, vhMdl, dt, est_mode):
    """ This is the measurement model to the kinematic<->sensor model above """
    y = [0]*7
    y[0] = x[0]   # x
    y[1] = x[1]   # y
    y[2] = x[2]   # vx
    y[3] = x[7]   # psiDot
    y[4] = x[4]   # a_x
    y[5] = x[5]   # a_y
    y[6] = x[3]   # vy
    return array(y)
