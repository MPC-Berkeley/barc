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

from input_map import angle_2_servo, servo_2_angle
from numpy import sin, cos, pi

# simple test setting class
class TestSettings:
	def __init__(self, SPD = 95, turn = 0, dt = 10):
		# PWN signal values for motor
		self.speed 		= SPD
		self.neutral 	= 90
		self.stopped 	= False
		self.brake 		= 50
		self.dt_man 	= dt   	# duration of manuever

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
def CircularTest(test_opt, rate, t_i):
    oneSec 		= rate
    t_0         = 3*oneSec
    t_f         = t_0 + (test_opt.dt_man)*oneSec

    # do nothing initially
    if (t_i < t_0):
        motorCMD        = test_opt.neutral
        servoCMD         = test_opt.Z_turn

    # turn left and move
    elif (t_i >= t_0) and (t_i <= t_f):
        servoCMD     = test_opt.turn
        motorCMD    = test_opt.speed

    # set straight and stop
    else:
        servoCMD     	= test_opt.Z_turn
        motorCMD        = test_opt.neutral

    return (motorCMD, servoCMD)


#############################################################
def Straight(test_opt, rate, t_i):
    # timing maneuvers
    oneSec      = rate
    dt          = (test_opt.dt_man)*oneSec
    t_0         = 5*oneSec
    t_f         = t_0 + dt

    # rest
    if t_i < t_0:
        servoCMD     = test_opt.Z_turn
        motorCMD    = test_opt.neutral

    # start moving
    elif (t_i >= t_0) and (t_i < t_f):
        servoCMD     = test_opt.Z_turn
        motorCMD    = test_opt.speed

    # set straight and stop
    else:
        servoCMD     	= test_opt.Z_turn
        motorCMD        = test_opt.neutral
        if not test_opt.stopped:
            motorCMD    	 = test_opt.brake
	

    return (motorCMD, servoCMD)


#############################################################
def CoastDown(test_opt, rate, t_i):
    # timing maneuvers
    oneSec      = rate
    dt          = (test_opt.dt_man)*oneSec
    t_0         = 5*oneSec
    t_f         = t_0 + dt

    # rest
    if t_i < t_0:
        servoCMD     = test_opt.Z_turn
        motorCMD    = test_opt.neutral

    # start moving
    elif (t_i >= t_0) and (t_i < t_f):
        servoCMD     = test_opt.Z_turn
        motorCMD    = test_opt.speed

    # set straight and stop
    else:
        servoCMD     	= test_opt.Z_turn
        motorCMD        = test_opt.neutral

    return (motorCMD, servoCMD)



#############################################################
def SineSweep(test_opt, rate, t_i):
    # timing maneuvers
    oneSec      	= rate
    dt          	= 15*oneSec
    start_turning 	= 1*oneSec

    t_0         	= 5*oneSec
    t_st         	= t_0 + start_turning
    t_f         	= t_0 + dt +start_turning
    T           	= 2*oneSec

    # rest
    if t_i < t_0:
        servoCMD     = test_opt.Z_turn
        motorCMD    = test_opt.neutral
	
	# move forward
    elif  (t_i >= t_0) and (t_i < t_st):
        servoCMD     = test_opt.Z_turn
        motorCMD    = test_opt.speed

	# move in sine wave motion
    elif  (t_i >= t_st) and (t_i < t_f):
        servoCMD     = angle_2_servo(15*sin(2*pi*(t_i-t_st)/float(T)))
        motorCMD    = test_opt.speed

    # set straight and stop
    else:
        servoCMD     	= test_opt.Z_turn
        motorCMD        = test_opt.neutral
        if not test_opt.stopped:
            motorCMD    	 = test_opt.brake

    return (motorCMD, servoCMD)

#############################################################
def DoubleLaneChange(test_opt, rate, t_i):
    # timing maneuvers
    oneSec      = rate
    dt          = 3*oneSec
    t_0         = 5*oneSec
    t_LT        = t_0 + dt
    t_RT        = t_LT + dt
    t_ZT        = t_RT + dt
    t_f         = t_ZT + dt

    # start moving
    if t_i < t_0:
        servoCMD     = test_opt.Z_turn
        motorCMD    = test_opt.speed

    # turn left
    elif (t_i >= t_0) and (t_i < t_LT):
        servoCMD    = abs(test_opt.turn)
        motorCMD    = test_opt.speed

    # turn right
    elif (t_i >= t_LT) and (t_i < t_RT):
        servoCMD    = -abs(test_opt.turn)
        motorCMD    = test_opt.speed

    # go straight
    elif (t_i >= t_RT) and (t_i < t_ZT):
        servoCMD     = test_opt.Z_turn
        motorCMD    = test_opt.speed

    # set straight and stop
    else:
        servoCMD     	= test_opt.Z_turn
        motorCMD        = test_opt.neutral
        if not test_opt.stopped:
            motorCMD    	 = test_opt.brake

    return (motorCMD, servoCMD)
