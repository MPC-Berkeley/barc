#!/usr/bin/python
import rospy
from geometry_msgs.msg import Vector3, Twist
from data_service.msg import TimeData
from math import pi,sin
import time
import serial
from numpy import genfromtxt, zeros, hstack

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
data_file.write('t_s,roll,pitch,yaw,a_x,a_y,a_z,w_x,w_y,w_z,vx_est,vy_est,X_est,Y_est,d_F,FxR,vx_enc, enc_count\n')

# input commands
d_F 	= 0
FxR     = 0
vx_enc = 0
new_enc_pos = 0

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
def saveData_callback(data):
	all_data = (time.time(),) + tuple(data.value) + (d_F, FxR,vx_enc, new_enc_pos)
	data_file.write('%.3f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f, %.3f, %.3f, %.3f, %d\n' % all_data)

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
def LQR_drift(init_sequence, t_i, rate):

	oneSecCount = rate
	t_f 		= 7*oneSecCount 		# final time after ten seconds
	
	# determine length of initial opening drift maneuver
	N = init_sequence.size

	# if still at start, perform open loop control
	if t_i < N:
		d_f 	= init_sequence[t_i] * 180/pi
		TURNcmd  = angle_2_servo(d_f) 
		SPEEDcmd = SPD
	elif t_i < t_f:
		# u = K * x
		d_f 		= 0	
		TURNcmd  = angle_2_servo(d_f) 
		SPEEDcmd = SPD
	else:
		TURNcmd  = Z_TURN 
		SPEEDcmd    = NEUTRAL

	#print "time step ", t_i, " with speed " , SPEEDcmd, " and steering angle ", TURNcmd
	return (TURNcmd, SPEEDcmd)

#############################################################
def main_auto():
	rospy.init_node('auto_mode', anonymous=True)
	rospy.Subscriber('imu_data',TimeData, saveData_callback)
	enc_data_pub = rospy.Publisher('enc_data', Vector3, queue_size = 10)

    # specify tests
	test_opt    = { 0 : CircularTest,
                    1 : DoubleLaneChange,
                    2 : Straight,
		    		3 : SineSweep,   
					4 : LQR_drift }

	test_sel    = 4
	test_mode   = test_opt.get(test_sel)

	rateHz  = 10 
	rate 	= rospy.Rate(rateHz)		# set the rate to one Hert
	t_i     = 0
     
	global d_F, FxR, vx_enc, new_enc_pos

    # Old encoder Position
	old_enc_pos 	= 0
	vx_enc 			= 0
	radius  		= 0.036
   	dist_between_magnets = 2*pi*radius/4    # four magnets equally spaced along perimeter of tire rim

	# open loop drift maneuver - STEERING WHEEL COMMAND
	# driver straight, then perform manuever to enter drift
	drive_straight 			= zeros(rateHz * 2)
	start_drift_maneuver 	= 1.5*genfromtxt('startDriftManeuver',delimiter=',')
	initial_sequence 		= hstack((drive_straight, start_drift_maneuver)) 
	
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

		######################################################################
		# ENCODER CODE 
		#####################################################################

        # read encoders and estimate speed
		num_of_bytes = 12
		parsed_data = ser.read(num_of_bytes).splitlines()
		# print parsed_data

		# number of loop iterations between valid readings
		n_dt += 1

		# ensure read sizeable buffer
		if (len(parsed_data)>1):

			# get latest measurement
			latest_measurement = parsed_data[-2]

			# ensure latest measurement value is not corrupt
			if len(latest_measurement) != 0: 

				# assuming that the car moves only forward !!!!
				latest_measurement = int(latest_measurement)
				if latest_measurement >= new_enc_pos:
					new_enc_pos = latest_measurement
					# time elapsed
					dt 			= n_dt / rateHz

					# dividing by two since measurement is a sum of two encoder readings 
					vx_enc = (new_enc_pos - old_enc_pos)*dist_between_magnets/(2*dt)

					# update old encoder position, reset dt counter
					old_enc_pos = new_enc_pos
					n_dt = 0.0
					# print "avg position = ", new_enc_pos
					# print "velocity = ", vx_enc

		#############################################################################
		# END ENCODER CODE
		#############################################################################

		# publish speed and steering angle
		print d_F
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
