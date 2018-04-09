#!/usr/bin/env python

# Author: Tony Zheng
# MEC231A BARC Project 

import rospy
import time
from geometry_msgs.msg import Twist
from barc.msg import ECU, Input, Moving



def start_callback(data):
    global move, still_moving
    #print("2")
    #print(move)
    if data.linear.x >0:
        move = True
    elif data.linear.x <0:
        move = False
    #print("3")
    #print(move)
    pubname.publish(newECU)

def moving_callback_function(data):
    global still_moving, move
    if data.moving == True:
        still_moving = True
    else:
        move = False
        still_moving = False

# update
def callback_function(data):
    global move, still_moving
    ################################################################################################################################################
    # Convert the velocity into motorPWM and steering angle into servoPWM
    newECU.motor = 
    newECU.servo =  #################################################################################################################################################

    maxspeed = 1568
    minspeed = 1400
    servomax = 1800
    servomin = 1200
    if (newECU.motor<minspeed):
        newECU.motor = minspeed
    elif (newECU.motor>maxspeed):
        newECU.motor = maxspeed
    if (newECU.servo<servomin):
        newECU.servo = servomin
    elif (newECU.servo>servomax):
        newECU.servo = servomax     # input steering angle

    if ((move == False) or (still_moving == False)):
        newECU.motor = 1500
        newECU.servo = 1550
    #print("5")
    #print(move)

    pubname.publish(newECU)
# state estimation node
def inputToPWM():
    
    # initialize node
    rospy.init_node('inputToPWM', anonymous=True)
    
    global pubname , newECU , subname, move , still_moving
    newECU = ECU() 
    newECU.motor = 1500
    newECU.servo = 1550
    move = False
    still_moving = False
    #print("1")
    #print(move)
    # topic subscriptions / publications
    pubname = rospy.Publisher('ecu_pwm',ECU, queue_size = 2)
    rospy.Subscriber('turtle1/cmd_vel', Twist, start_callback)
    subname = rospy.Subscriber('uOpt', Input, callback_function)
    rospy.Subscriber('moving', Moving, moving_callback_function)
    # set node rate
    loop_rate   = 40
    ts          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()
     
    rospy.spin()



if __name__ == '__main__':
    try:
       inputToPWM()
    except rospy.ROSInterruptException:
        pass
