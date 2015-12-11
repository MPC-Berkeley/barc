def CircularTest(ctrl_mdl, t_i, rate):
    oneSec 	= rate
    t_0         = 3
    t_f         = t_0 + 5*oneSec

    # set neutral initially
    if (t_i < t_0):
		ctrl_mdl.setNeutral()
		print "neutral"
    # turn left and move
    elif (t_i >= t_0) and (t_i <= t_f):
		ctrl_mdl.turnLeft()
		print "turning now"
    # set straight and stop
    else:
		ctrl_mdl.setBrake()

#############################################################
def Straight(ctrl_mdl, t_i, rate):
    # timing maneuvers
    oneSec      = rate
    dt          = 2*oneSec 
    t_0         = 2
    t_f         = t_0 + dt

    # set neutral initially
    if t_i < t_0:
		ctrl_mdl.setNeutral()
    # start moving straight
    elif (t_i >= t_0) and (t_i < t_f):
		ctrl_mdl.goStraight()
    # set straight and stop
    else:
		ctrl_mdl.setBrake()

#############################################################
def SineSweep(ctrl_mdl, t_i, rate):
    # timing maneuvers
    oneSec      = rate
    dt          = 2.5*oneSec 
    t_0         = 5*oneSec
    start_turning = 1*oneSec
    t_st         = t_0+start_turning
    t_f         = t_0 + dt +start_turning
    T           = 1.2*oneSec

    # set neutral initially
    if (t_i < t_0):
		ctrl_mdl.setNeutral()
    # move straight
    elif (t_i >= t_0) and (t_i < t_f):
		ctrl_mdl.goStraight()
	# move in a sine wave
    elif  (t_i >= t_st) and (t_i < t_f):
		turnCMD  	=	angle_2_servo(25*sin(2*pi*(t_i-t_0)/float(T)))
		ctrl_mdl.send_cmd(df_PWM = turnCMD)
    # set straight and stop
    else:
		ctrl_mdl.setBrake()

#############################################################
def DoubleLaneChange(ctrl_mdl, t_i, rate):
    # timing maneuvers
    oneSecCounts    = rate 				# number of counts in a second
    dt          	= 2*oneSecCounts 
    t_0         	= 2
    t_LT        	= t_0 + dt
    t_RT        	= t_LT + dt
    t_ZT        	= t_RT + dt
    t_f         	= t_ZT + dt

    # set neutral initially
    if (t_i < t_0):
		ctrl_mdl.setNeutral()
    # turn left
    elif (t_i >= t_0) and (t_i < t_LT):
		ctrl_mdl.turnLeft()
    # turn right
    elif (t_i >= t_LT) and (t_i < t_RT):
		ctrl_mdl.turnRight()
    # go straight
    elif (t_i >= t_RT) and (t_i < t_ZT):
		ctrl_mdl.goStraight()
    # set straight and stop
    else:
		ctrl_mdl.setBrake()


