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

"""
Mappings from PWM signal to servo angle, and vice verca
"""

# [deg] -> [PWM]
def angle_2_servo(x):
    u   = 92.0558 + 1.8194*x  - 0.0104*x**2
    return u

# [PWM] -> [deg]
def servo_2_angle(x):
    d_f   = -(39.2945 - 0.3013*x  - 0.0014*x**2)
    return d_f
