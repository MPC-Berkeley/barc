# fixed or variable sampling time 1/0
fixTime = 0		# default: 0 (variable time steps)

#### problem parameters ####
TsPF = 0.05
# nominal sampling time
sampleN = 3 		# down-sampling from Hybrid A* to OBCA
if fixTime == 1 	# 
	Ts = 0.65/3*sampleN		# 0.65/3 must be compatible with motion resolution of Hybrid A* algorithm
else
	Ts = 0.6/3*sampleN 		# 0.6/3 must be compatible with motion resolution of Hybrid A* algorithm
end

# wheelbase
#L  = 2.7            # wheel base length
motionStep = 0.1	# step length of Hybrid A*",

# "nominal" shape of ego/controlled car,  ego object is later rotated around the car center
# center of rear wheel axis is reference point
# size of car is: (x_upper + x_lower) + (y_upper + y_lower)
#	   [x_upper, y_upper, -x_lower, -y_lower ]
ego  = [ 3.7   , 1      ,  1      ,  1       ]