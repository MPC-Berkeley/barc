#!/usr/bin/python
import rospy
from geometry_msgs.msg import Vector3, Twist
from pyfirmata import Arduino
from data_service.msg import TimeData
from math import pi
import time

# communication scheme
# ODROID -> Arduino -> actuators (servo, motor)
SERVO_MODE  = 4    		# servo mode, sends PWM signals
ESC_PIN     = 3   		# motor (throttle) pin
SRV_PIN     = 5  		# servo (steering wheel) pin

SPD         = 98       	        # initial speed PWN signal value
BRAKE       = 50         	# brake PWN signal value
NEUTRAL     = 90         	# neutral PWN signal value
STOPPED     = False

board       = Arduino('/dev/ttyACM99') 	# connect to arduino
board.digital[ESC_PIN]._set_mode(SERVO_MODE)
board.digital[SRV_PIN]._set_mode(SERVO_MODE)
board.digital[ESC_PIN].write(90)
board.digital[SRV_PIN].write(90)

# open file for data collection
BASE_PATH   		= "/home/odroid/Data/"
data_file_name   	= BASE_PATH + time.strftime("%Y-%m-%d_%H-%M-%S") + '.csv'
data_file     		= open(data_file_name, 'a')
data_file.write('t_s,roll,pitch,yaw,a_x,a_y,a_z,w_x,w_y,w_z,vx_est,vy_est,X_est,Y_est,d_F,FxR\n')

# input commands
d_F 	= 0
FxR     = 0

#############################################################
# [deg] -> [PWM]
def angle_2_servo(x):
    u   = 92.0558 - 1.8194*x  - 0.0104*x**2
    return u

# [PWM] -> [deg]
def servo_2_angle(x):
    d_F   = 39.2945 - 0.3018*x  - 0.0014*x**2
    return d_F

# Note: left turns
L_TURN 	    = angle_2_servo(20)
R_TURN 	    = angle_2_servo(-20)
Z_TURN 	    = angle_2_servo(0)

#############################################################
def saveData_callback(data):
	all_data = (time.time(),) + tuple(data.value) + (d_F, FxR)
	data_file.write('%.3f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f, %.2f, %.2f\n' % all_data)

#############################################################
def CircularTest(t_i):
    t_0         = 3
    t_f         = t_0 + 5

    # do nothing initially
    if (t_i < t_0):
        SPEEDcmd        = NEUTRAL
        TURNcmd         = Z_TURN
    # turn left and move
    elif (t_i >= t_0) and (t_i <= t_f):
        TURNcmd     = R_TURN
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
def DoubleLaneChange(t_i):
    # timing maneuvers
    dt          = 1
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
def main_auto():
    rospy.init_node('auto_mode', anonymous=True)
    rospy.Subscriber('imu_data',TimeData, saveData_callback)

    # specify tests
    test_opt    = { 0 : CircularTest,
                    1 : DoubleLaneChange}
    test_sel    = 1
    test_mode   = test_opt.get(test_sel)

    rate 	= rospy.Rate(10)		# set the rate to one Hert
    t_i         = 0

    global d_F, FxR

    while not rospy.is_shutdown():
        # determine inputs
        (TURNcmd, SPEEDcmd) = test_mode(t_i)
	d_F 	= servo_2_angle(TURNcmd)
	FxR     = SPEEDcmd

        # send command signal
        board.digital[SRV_PIN].write(TURNcmd)
        board.digital[ESC_PIN].write(SPEEDcmd)

        # increment counter, and wait
        t_i += 1
        rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        main_auto()
    except rospy.ROSInterruptException:
        pass
