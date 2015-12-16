#!/usr/bin/python
import rospy
from geometry_msgs.msg import Vector3, Twist
from barc.msg import TimeData
from math import pi,sin
import time
import serial
from numpy import genfromtxt, zeros, hstack, cos, array, dot, arctan

# communication scheme
# ODROID -> Arduino -> actuators (servo, motor)
SPD         = 99       	        # initial speed PWN signal value
BRAKE       = 50         	# brake PWN signal value NEUTRAL     = 90         	# neutral PWN signal value 
NEUTRAL     = 90         	# brake PWN signal value NEUTRAL     = 90         	# neutral PWN signal value 
STOPPED     = False

# Initialize serial connection
ser = serial.Serial(port = '/dev/ttyUSB0',baudrate = 115200)
if ser.isOpen():
    ser.close()
ser.open()
ser.write("90,90f") #for phyton 3 useser.write("90,90f".encode())

# open file for data collection
BASE_PATH   		= "/home/odroid/Data/"
data_file_name   	= BASE_PATH + time.strftime("%Y-%m-%d_%H-%M-%S") + '.csv'
data_file     		= open(data_file_name, 'a')
data_file.write('t_s,roll,pitch,yaw,a_x,a_y,a_z,w_x,w_y,w_z,d_F,FxR,vx_enc,enc_count\n')

# input command signals
d_F 		= 0
FxR     	= 0

# encoder measurement data
vx_enc 		= 0
old_enc_pos = 0
new_enc_pos = 0
radius  		= 0.036
dist_between_magnets = 2*pi*radius/4    # four magnets equally spaced along perimeter of tire rim

# state estimate
x_hat = zeros(3)

# open loop drift maneuver - STEERING WHEEL COMMAND
# driver straight, then perform manuever to enter drift
rateHz  = 50						# same rate as imu_data collection 
drive_straight 			= zeros(rateHz * 2)
dir_path = '/home/odroid/catkin_ws/src/barc/data'
start_drift_maneuver 	= genfromtxt(dir_path + '/startDriftManeuver',delimiter=',')
initial_sequence 		= hstack((drive_straight, start_drift_maneuver)) 

# LQR gain
K_lqr = genfromtxt(dir_path + '/K_lqr', delimiter=',') 

#############################################################
# [deg] -> [PWM]
def angle_2_servo(x):
    u   = 92.0558 + 1.8194*x  - 0.0104*x**2
    return u

# [PWM] -> [deg]
def servo_2_angle(x):
    d_f   = -(39.2945 - 0.3013*x  - 0.0014*x**2)
    return d_f
L_TURN 	    = angle_2_servo(10) 
R_TURN 	    = angle_2_servo(-10)
Z_TURN 	    = angle_2_servo(0)

#############################################################
# Save IMU data to disk
# record time stamp, imu estimates, and inputs
def saveData_callback(imu_data):
	all_data = (time.time(),) + tuple(imu_data.value) + (d_F, FxR, vx_enc, new_enc_pos)
	N = len(all_data)
	str_fmt = '%.4f,'*N 
	data_file.write((str_fmt[0:-1] + '\n') % all_data)

#############################################################
def CircularTest(t_i, rate):
    oneSec 	= rate
    t_0         = 3
    t_f         = t_0 + 3*oneSec

    # do nothing initially
    if (t_i < t_0):
        SPEEDcmd        = NEUTRAL
        TURNcmd         = Z_TURN
    # turn left and move
    elif (t_i >= t_0) and (t_i <= t_f):
        TURNcmd     = angle_2_servo(25)
        SPEEDcmd    = SPD
    # set straight and stop
    else:
        TURNcmd     = Z_TURN
        SPEEDcmd        = NEUTRAL
        global STOPPED
        if not STOPPED:
            SPEEDcmd    = BRAKE
            STOPPED     = True

    return (TURNcmd, SPEEDcmd)

#############################################################
def Straight(t_i, rate):
    # timing maneuvers
    oneSec      = rate
    dt          = 2*oneSec 
    t_0         = 2
    t_f         = t_0 + dt

    # stop moving
    if t_i < t_0:
        TURNcmd     = 92
        SPEEDcmd    = 90
    # start moving
    elif (t_i >= t_0) and (t_i < t_f):
        TURNcmd     = 92
        SPEEDcmd    = 95
    # stop
    else:
        TURNcmd     = Z_TURN
        SPEEDcmd    = NEUTRAL
        global STOPPED
        if not STOPPED:
            SPEEDcmd    = BRAKE
            STOPPED     = True

    return (TURNcmd, SPEEDcmd)

#############################################################
def SineSweep(t_i, rate):
    # timing maneuvers
    oneSec      = rate
    dt          = 2.5*oneSec 
    t_0         = 5*oneSec
    start_turning = 1*oneSec
    t_st         = t_0+start_turning
    t_f         = t_0 + dt +start_turning
    T           = 1.2*oneSec

    # start moving
    if  (t_i<t_0):
        TURNcmd     = Z_TURN
        SPEEDcmd    = NEUTRAL
    elif  (t_i >= t_0) and (t_i < t_st):
        TURNcmd     = Z_TURN
        SPEEDcmd    = SPD	
    elif  (t_i >= t_st) and (t_i < t_f):
        TURNcmd     = angle_2_servo(25*sin(2*pi*(t_i-t_0)/float(T)))
        SPEEDcmd    = SPD
    # stop
    else:
        TURNcmd     = Z_TURN
        SPEEDcmd    = BRAKE
 

    return (TURNcmd, SPEEDcmd)


#############################################################
def DoubleLaneChange(t_i, rate):
    # timing maneuvers
    oneSec      = rate
    dt          = 2*oneSec 
    t_0         = 2
    t_LT        = t_0 + dt
    t_RT        = t_LT + dt
    t_ZT        = t_RT + dt
    t_f         = t_ZT + dt

    # start moving
    if t_i < t_0:
        TURNcmd     = Z_TURN
        SPEEDcmd    = SPD
    # turn left
    elif (t_i >= t_0) and (t_i < t_LT):
        TURNcmd     = L_TURN
        SPEEDcmd    = SPD
    # turn right
    elif (t_i >= t_LT) and (t_i < t_RT):
        TURNcmd     = R_TURN
        SPEEDcmd    = SPD
    # go straight
    elif (t_i >= t_RT) and (t_i < t_ZT):
        TURNcmd     = Z_TURN
        SPEEDcmd    = SPD
    # stop
    else:
        TURNcmd     = Z_TURN
        SPEEDcmd    = NEUTRAL
        global STOPPED
        if not STOPPED:
            SPEEDcmd    = BRAKE
            STOPPED     = True

    return (TURNcmd, SPEEDcmd)


#############################################################
# get estimate of x_hat = [v_x , v_y, w_z]
def updateState_callback(data):
	global x_hat

	# update fields
	x_hat[0] = data.x 		# v_x  longitudinal velocity 
	x_hat[1] = data.y 		# v_y  lateral velocity
	x_hat[2] = data.z		# w_z  angular velocity about z-axis

#############################################################
def LQR_drift(init_sequence, t_i, rate):

	# initial setting
	oneSecCount = rate
	t_f 		= 7*oneSecCount 		# final time
	N 			= init_sequence.size 	# length of opening sequence

	# get current state estimate
	global x_hat, K_lqr
	v_x 	= x_hat[0]
	v_y 	= x_hat[1]
	w_z 	= x_hat[2]

	# compute slip angle beta
	if v_x == 0:
		beta = 0
	else:
		beta  	= arctan(v_y/v_x)

	# define lqr state [beta, w_z, v_x]
	x_lqr 	= array([beta, w_z, v_x])

	# OPEN LOOP CONTROL 
	if t_i < N:
		d_f 	= init_sequence[t_i] * 180/pi
		TURNcmd  = angle_2_servo(d_f) 
		SPEEDcmd = SPD

	# FEEDBACK CONTROL
	elif t_i < t_f:
		# compute LQR gain
		u 		= dot(K_lqr, x_lqr)

		# extract individual inputs
		d_f 		= u[0]*180/pi	
		v_x_ref 	= u[1]
		
		# TO DO -- NEED MAPPING FROM v_x_ref to ESC PWM signal
		TURNcmd  = angle_2_servo(d_f) 
		SPEEDcmd = 0 
	
	# END EXPERIMENT
	else:
		TURNcmd  = Z_TURN 
		SPEEDcmd    = NEUTRAL

	#print "time step ", t_i, " with speed " , SPEEDcmd, " and steering angle ", TURNcmd
	return (TURNcmd, SPEEDcmd)

#############################################################
def main_auto():
	# initial ROS node
	rospy.init_node('auto_mode', anonymous=True)
	rospy.Subscriber('imu_data',TimeData, saveData_callback)
	rospy.Subscriber('state_estimate', Vector3, updateState_callback)
	enc_data_pub = rospy.Publisher('enc_data', Vector3, queue_size = 10)

    # specify tests
	test_opt    = { 0 : CircularTest,
                    1 : DoubleLaneChange,
                    2 : Straight,
		    		3 : SineSweep,   
					4 : LQR_drift }

	test_sel    = 4
	test_mode   = test_opt.get(test_sel)

	rateHz  = 50						# same rate as imu_data collection 
	rate 	= rospy.Rate(rateHz)		# set the rate to one Hert
	t_i     = 0
     
	global d_F, FxR, vx_enc, new_enc_pos

    # Old encoder Position
	old_enc_pos 	= 0
	vx_enc 			= 0
	radius  		= 0.036
   	dist_between_magnets = 2*pi*radius/4    # four magnets equally spaced along perimeter of tire rim
	
	n_dt = 0.0
	while not rospy.is_shutdown():
		# determine test mode: open-loop or closed-loop
		if test_sel in [0,1,2,3]:
			(TURNcmd, SPEEDcmd) = test_mode(t_i, rateHz)
		else:
			(TURNcmd, SPEEDcmd) = test_mode(initial_sequence, t_i, rateHz)
			
		# get input commands 
		d_F 	= servo_2_angle(TURNcmd)  # [deg]
		FxR     = SPEEDcmd

        # send command signal 
		ESC_CMD = str(SPEEDcmd)+','+str(int(TURNcmd))+'f'
		ser.write( ESC_CMD ) 
	
		###################################################################
		# ENCODER CODE
		##################################################################

		# send command to get encoder data
		num_of_bytes = 12
		parsed_data = ser.read(num_of_bytes).splitlines()

		# number of loop iterations between valid readings
		n_dt += 1

		print "here"
		# ensure read sizeable buffer
		if (len(parsed_data)>1):

			# get latest measurement
			latest_measurement = parsed_data[-2]

			# ensure latest measurement value is not corrupt
			if len(latest_measurement) != 0: 

				# assuming that the car moves only forward !!!!
				latest_measurement = int(latest_measurement)
				if latest_measurement >= new_enc_pos:
					# time elapsed
					dt 			= n_dt / rateHz

					# dividing by two since measurement is a sum of two encoder readings 
					# project velocity onto vehicle x-axis
					new_enc_pos = latest_measurement
					v_enc = (new_enc_pos - old_enc_pos)*dist_between_magnets/(2*dt)
					vx_enc = v_enc*cos(d_F*pi/180)

					# update old encoder position, reset dt counter
					old_enc_pos = new_enc_pos
					n_dt = 0.0

		###################################################################
		# END ENCODER CODE
		##################################################################

		# publish speed and steering angle
		enc_data_pub.publish( Vector3(vx_enc, 0, d_F*pi/180 ) )
                
        # increment counter, and wait
		t_i += 1 
		rate.sleep()

#############################################################
if __name__ == '__main__':
	print('starting test ...')
	try:
		main_auto()
	except rospy.ROSInterruptException:
		pass
