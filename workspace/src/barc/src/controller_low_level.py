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
from barc.msg import ECU, Z_KinBkMdl, vel_sgn
from numpy import pi, abs
import rospy

motor_pwm = 90
servo_pwm = 90
vel = 0
mode = 1
time = 0
def pwm_converter_callback(msg):
    global motor_pwm, servo_pwm, b0, mode, time
    # translate from SI units in vehicle model
    # to pwm angle units (i.e. to send command signal to actuators)

    rate = rospy.Rate(30)
    # compute servo command
    str_ang     = 180.0/pi*msg.servo  # convert steering angle to degrees
    servo_pwm   = 92.0558 + 1.8194*str_ang  - 0.0104*str_ang**2

    # compute motor command
    FxR         =  float(msg.motor) 
    if FxR == 0:
        motor_pwm = 90
    elif FxR > 0:
        if abs(vel) < 0.01 and mode == -1:
            mode = 1
            sign = vel_sgn(mode)
            pub_sgn.publish(sign)            
        motor_pwm   =  FxR/b0 + 96.5
    elif FxR < 0:
        if abs(vel) < 0.01 and mode == -1:
            time+=1
            if time > 30: # 1 sec based on Rate
                mode = 1
                time = 0
        if abs(vel) < 0.01 and mode == 1:
            mode = -1
            sign = vel_sgn(mode)
            pub_sgn.publish(sign)

            # need this sequence to change to backward mode
            i = 0
            j = 0
            latched = 0
            while i < 40:
                motor_pwm = 65
                update_arduino()
                rate.sleep()
                if abs(vel) > 0.1:
                    latched = 1
                    break
                i+=1
            if latched == 0:
                while j < 40:
                    motor_pwm = 90
                    update_arduino()
                    rate.sleep()
                    j+=1
        motor_pwm   =  FxR/b0 + 87
        update_arduino()    
    else:
        motor_pwm = 90
    update_arduino()

def SE_callback(msg):
    global vel
    vel =  msg.v

def neutralize():
    global motor_pwm
    motor_pwm = 90
    servo_pwm = 90
    update_arduino()

def update_arduino():
    global motor_pwm, servo_cmd, ecu_pub
    ecu_cmd = ECU(motor_pwm, servo_pwm)
    ecu_pub.publish(ecu_cmd)

def arduino_interface():
    global ecu_pub, pub_sgn, b0, time

    # launch node, subscribe to motorPWM and servoPWM, publish ecu
    init_node('arduino_interface')
    b0  = get_param("input_gain")

    Subscriber('ecu', ECU, pwm_converter_callback, queue_size = 10)
    Subscriber('state_estimate', Z_KinBkMdl, SE_callback, queue_size = 10)
    pub_sgn = Publisher('vel_sgn', vel_sgn, queue_size = 10)
    ecu_pub = Publisher('ecu_pwm', ECU, queue_size = 10)

  #  rate = rospy.Rate(10)

  #  if abs(vel) <= 0.05:
  #      time += 0.1
  #  if abs(vel) > 0.05:
  #      time = 0
  #  if time > 50:
  #      k = 0
  #      l = 0
  #      while k < 50:
  #          motor_pwm = 60
  #          update_arduino()
  #          rate.sleep()
  #          k+=1
  #      while l < 50:
  #          motor_pwm = 90
  #          update_arduino()
  #          rate.sleep()
  #          l+=1
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
