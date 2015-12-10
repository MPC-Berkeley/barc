#!/usr/bin/python
import rospy
from geometry_msgs.msg import Vector3, Twist
from pyfirmata import Arduino
from numpy import pi
import time

# communication scheme
# ODROID -> Arduino -> actuators (servo, motor)
SERVO_MODE  = 4    			# servo mode, sends PWM signals
ESC_PIN     = 3   			# motor (throttle) pin
SRV_PIN     = 5  			# servo (steering wheel) pin

SPD         = 100        	# initial speed PWN signal value
BRAKE       = 50         	# brake PWN signal value
NEUTRAL     = 90         	# neutral PWN signal value
L_TURN 	    = 50 	    	# initial steering angle signal value

board       = Arduino('/dev/ttyACM99') 	# connect to arduino
board.digital[ESC_PIN]._set_mode(SERVO_MODE)
board.digital[SRV_PIN]._set_mode(SERVO_MODE)

#############################################################
def servo_cmd_callback(data):
    # get steering angle
    # compute servo signal
    # send servo signal
    x   = (180/pi)*data.x
    u   = 92.0558 - 1.8194*x  - 0.0104*x**2
    board.digital[SRV_PIN].write(int(u))

#############################################################
def manual_cmd_callback(data):
    # increase speed if user presses key 'i'
    if (data.linear.x == 0.5) and (data.angular.z == 0):
        global SPD
        SPD += 3
        board.digital[ESC_PIN].write(SPD)

    # decrease speed if user presses key ','
    elif (data.linear.x == -0.5) and (data.angular.z == 0):
        global SPD
        SPD -= 3
        board.digital[ESC_PIN].write(SPD)

    # otherwise BRAKE!!!
    else:
        board.digital[ESC_PIN].write(BRAKE)
        time.sleep(2)
        board.digital[ESC_PIN].write(NEUTRAL)
        board.digital[SRV_PIN].write(NEUTRAL)

#############################################################
def main_auto():
    rospy.init_node('auto_mode', anonymous=True)
    rospy.Subscriber('NMPC_ctrl_sig', Vector3, servo_cmd_callback)
    rospy.Subscriber('/cmd_vel', Twist, manual_cmd_callback)

    rate = rospy.Rate(1)		# set the rate to one hert
    t_rest  = 10
    t_i     = 0

    while not rospy.is_shutdown():
        # rest the car to initialize sensors,
        # then run car in a circle
        if t_i == t_rest:
                board.digital[SRV_PIN].write(L_TURN)
                board.digital[ESC_PIN].write(SPD)

        # increment counter
        t_i += 1
        rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        main_auto()
    except rospy.ROSInterruptException:
        pass
