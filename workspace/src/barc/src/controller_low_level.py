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

motor_pwm = 90
servo_pwm = 90
str_ang_max = 35
str_ang_min = -35

def pwm_converter_callback(msg):
    global motor_pwm, servo_pwm
    global str_ang_max, str_ang_min

    # translate from SI units in vehicle model
    # to pwm angle units (i.e. to send command signal to actuators)

    # convert desired steering angle to degrees, saturate based on input limits
    servo_pwm       = 91.365 + 105.6*float(msg.servo)

    # compute motor command
    FxR         =  float(msg.motor) 
    if FxR == 0:
        motor_pwm = 90.0
    elif FxR > 0:
        motor_pwm   = 94.14 + 2.7678*FxR
    else:               # motor break / slow down
        motor_pwm = 93.5 + 46.73*FxR
    update_arduino()

def neutralize():
    global motor_pwm
    motor_pwm = 90
    servo_pwm = 90
    update_arduino()

def update_arduino():
    global motor_pwm, servo_pwm, ecu_pub
    ecu_cmd = ECU(motor_pwm, servo_pwm)
    ecu_pub.publish(ecu_cmd)

def arduino_interface():
    global ecu_pub

    # launch node, subscribe to motorPWM and servoPWM, publish ecu
    init_node('arduino_interface')

    Subscriber('ecu', ECU, pwm_converter_callback, queue_size = 1)
    ecu_pub = Publisher('ecu_pwm', ECU, queue_size = 1)

    # Set motor to neutral on shutdown
    on_shutdown(neutralize)

    # process callbacks and keep alive
    spin()

#############################################################
if __name__ == '__main__':
    try:
        arduino_interface()
    except ROSInterruptException:
        pass
