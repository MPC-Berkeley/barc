#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed at UC
# Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu) and Greg Marcil (grmarcil@berkeley.edu). The cloud
# services integation with ROS was developed by Kiet Lam
# (kiet.lam@berkeley.edu). The web-server app Dator was based on an open source
# project by Bruce Wootton
# ---------------------------------------------------------------------------

# README: This node serves as an outgoing messaging bus from odroid to arduino
# Subscribes: steering and motor commands on 'ecu'
# Publishes: combined ecu commands as 'ecu_pwm'

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from barc.msg import ECU
from numpy import pi
import rospy
import time

motor_pwm = 1500
servo_pwm = 1580

def arduino_interface():
    global ecu_pub, motor_pwm, servo_pwm

    init_node('arduino_interface')
    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    
    time_prev = time.time()
    ecu_pub = Publisher('ecu_pwm', ECU, queue_size = 10)

    while not rospy.is_shutdown():
        if time.time() >= time_prev and time.time() < time_prev + 7: 
            motor_pwm = 1620
        if time.time() >= time_prev + 5 and time.time() < time_prev + 12: 
            motor_pwm = 1440 #1465
        #if time.time() >= time_prev + 5 and time.time() < time_prev + 10: 
         #   motor_pwm = 84.0
        #if time.time() >= time_prev + 12 and time.time() < time_prev + 17: 
         #   motor_pwm = 86.0
        if time.time() >= time_prev + 17:
            motor_pwm = 1500
            ecu_cmd = ECU(motor_pwm, servo_pwm)
            ecu_pub.publish(ecu_cmd)
            break

        ecu_cmd = ECU(motor_pwm, servo_pwm)
        ecu_pub.publish(ecu_cmd)

        # wait
        rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        arduino_interface()
    except ROSInterruptException:
        pass
