#!/usr/bin/python

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
