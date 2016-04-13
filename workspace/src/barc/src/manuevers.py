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

from input_map import angle_2_servo, servo_2_angle
from numpy import sin, cos, pi
import numpy as np

# simple test setting class
class TestSettings:
    def __init__(self, SPD = 95, turn = 0, dt = 10):
		# PWN signal values for motor
        self.speed 		= SPD
        self.neutral 	= 90
        self.stopped 	= False
        self.brake 		= 50
        self.dt_man 	= dt   	# length of time the motor is on
        self.t_turn     = 4     # length of time before first turn
        self.t_0        = 2     # initial time at rest before experiment
        self.turn_deg   = turn

		# check valid speed
        if SPD < 90 or SPD > 130:
            self.speed = 95

		# check valid turns
		# left is positive, right is negative
        if turn < -30 or turn > 30:
            turn = 0

		# PWN signal values for servo
        self.turn 	    = angle_2_servo(turn)		# right turn
        self.Z_turn 	= 90 						# zero (no) turn


#############################################################
def CircularTest(opt, rate, t_i):
    oneSec 		= rate
    t_0         = opt.t_0*oneSec
    t_f         = t_0 + (opt.dt_man)*oneSec

    # do nothing initially
    if (t_i < t_0):
        motorCMD        = opt.neutral
        servoCMD         = opt.Z_turn

    # turn left and move
    elif (t_i >= t_0) and (t_i <= t_f):
        servoCMD     = opt.turn
        motorCMD    = opt.speed

    # set straight and stop
    else:
        servoCMD     	= opt.Z_turn
        motorCMD        = opt.neutral

    return (motorCMD, servoCMD)


#############################################################
def Straight(opt, rate, t_i):
    # timing maneuvers
    oneSec      = rate
    dt          = (opt.dt_man)*oneSec
    t_0         = opt.t_0*oneSec
    t_f         = t_0 + dt

    # rest
    if t_i < t_0:
        servoCMD     = opt.Z_turn
        motorCMD    = opt.neutral

    # start moving
    elif (t_i >= t_0) and (t_i < t_f):
        servoCMD     = opt.Z_turn
        motorCMD    = opt.speed

    # set straight and stop
    else:
        servoCMD     	= opt.Z_turn
        motorCMD        = opt.neutral

    return (motorCMD, servoCMD)


#############################################################
def SingleTurn(opt, rate, t_i):
    # timing maneuvers
    oneSec      = rate
    dt_motor    = (opt.dt_man)*oneSec
    t_turn     = (opt.t_turn)*oneSec
    t_0         = opt.t_0*oneSec

    # rest
    if t_i < t_0:
        servoCMD     = opt.Z_turn
        motorCMD    = opt.neutral

    # Motor command:
    # move
    if (t_i >= t_0) and (t_i < t_0 + dt_motor):
        step_up     = 95  + np.round( float(t_i - t_0) / float(rate) )
        motorCMD    = np.min([step_up, opt.speed])
    # stop
    else:
        motorCMD      = opt.neutral
        
    # go straight and then turn
    if (t_i <= t_0 + t_turn):
        servoCMD     = opt.Z_turn
    else:
        servoCMD     = opt.turn

    return (motorCMD, servoCMD)


#############################################################
def SingleHardTurn(opt, rate, t_i):
    # timing maneuvers
    oneSec      = rate
    dt_motor    = (opt.dt_man)*oneSec
    t_turn     = (opt.t_turn)*oneSec
    t_0         = opt.t_0*oneSec
    t_brake     = 2*oneSec

    # rest
    if t_i < t_0:
        servoCMD    = opt.Z_turn
        motorCMD    = opt.neutral

    # Motor command:
    # move
    if (t_i >= t_0) and (t_i < t_0 + dt_motor):
        step_up     = 95  + np.round( float(t_i - t_0) / float(rate) )
        motorCMD    = np.min([step_up, opt.speed])
    # stop
    elif (t_i <= t_0 + dt_motor + t_brake):
        motorCMD      = 30 # opt.brake
    elif (t_i <= t_0 + dt_motor + t_brake + 25):
        motorCMD      = opt.neutral
    elif (t_i <= t_0 + dt_motor + t_brake + 50):
        motorCMD      = 95
    else:
        motorCMD      = opt.neutral 
        
    # go straight and then turn
    if (t_i <= t_0 + t_turn):
        servoCMD     = opt.Z_turn
    else:
        servoCMD     = opt.turn

    return (motorCMD, servoCMD)

#############################################################
def CoastDown(opt, rate, t_i):
    # timing maneuvers
    oneSec      = rate
    dt          = (opt.dt_man)*oneSec
    t_0         = opt.t_0*oneSec
    t_f         = t_0 + dt

    # rest
    if t_i < t_0:
        servoCMD     = opt.Z_turn
        motorCMD    = opt.neutral

    # start moving
    elif (t_i >= t_0) and (t_i < t_f):
        servoCMD     = opt.Z_turn
        motorCMD    = opt.speed

    # set straight and stop
    else:
        servoCMD     	= opt.Z_turn
        motorCMD        = opt.neutral

    return (motorCMD, servoCMD)


#############################################################
def SineSweep(opt, rate, t_i):
    # timing maneuvers
    oneSec      	= rate
    dt          	= 15*oneSec
    start_turning 	= 1*oneSec

    t_0         = opt.t_0*oneSec
    t_st         	= t_0 + start_turning
    t_f         	= t_0 + dt +start_turning
    T           	= 2*oneSec

    # rest
    if t_i < t_0:
        servoCMD     = opt.Z_turn
        motorCMD    = opt.neutral
	
	# move forward
    elif  (t_i >= t_0) and (t_i < t_st):
        servoCMD     = opt.Z_turn
        motorCMD    = opt.speed

	# move in sine wave motion
    elif  (t_i >= t_st) and (t_i < t_f):
        servoCMD     = angle_2_servo(15*sin(2*pi*(t_i-t_st)/float(T)))
        motorCMD    = opt.speed

    # set straight and stop
    else:
        servoCMD     	= opt.Z_turn
        motorCMD        = opt.neutral
        if not opt.stopped:
            motorCMD    	 = opt.brake

    return (motorCMD, servoCMD)

#############################################################
def DoubleLaneChange(opt, rate, t_i):
    # timing maneuvers
    oneSec      = rate
    dt          = 3*oneSec
    t_0         = opt.t_0*oneSec
    t_LT        = t_0 + dt
    t_RT        = t_LT + dt
    t_ZT        = t_RT + dt
    t_f         = t_ZT + dt

    # start moving
    if t_i < t_0:
        servoCMD     = opt.Z_turn
        motorCMD    = opt.speed

    # turn left
    elif (t_i >= t_0) and (t_i < t_LT):
        servoCMD    = abs(opt.turn)
        motorCMD    = opt.speed

    # turn right
    elif (t_i >= t_LT) and (t_i < t_RT):
        servoCMD    = -abs(opt.turn)
        motorCMD    = opt.speed

    # go straight
    elif (t_i >= t_RT) and (t_i < t_ZT):
        servoCMD     = opt.Z_turn
        motorCMD    = opt.speed

    # set straight and stop
    else:
        servoCMD     	= opt.Z_turn
        motorCMD        = opt.neutral
        if not opt.stopped:
            motorCMD    	 = opt.brake

    return (motorCMD, servoCMD)
