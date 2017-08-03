#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Authors: J. Noonan and Jon Gonzales
# Emails: jpnoonan@berkeley.edu, jon.gonzales@berkeley.edu
#
# ---------------------------------------------------------------------------

from rospy import init_node, Subscriber, Publisher, get_param, on_shutdown
from rospy import Rate, is_shutdown, ROSInterruptException
from barc.msg import ECU, Encoder
from sensor_msgs.msg import Imu
from math import pi
from numpy import zeros, array
from numpy import unwrap
from tf import transformations
from pid import PID
import numpy as np
import rospy

import matplotlib 
matplotlib.use('Agg')
import matplotlib.pyplot as plt



global yaw_error, yaw_curr, yaw_vals, read_yaw0, t_vals, yaw_target

yaw_vals = []
t_vals = []
yaw_target = 90
yaw_curr = 90
read_yaw0 = False

def servo_callback(data):
    global yaw_error, yaw_curr, read_yaw0
    #2.8262693262x + 202.806447252
    #new_yaw = int((data.FL - 239.94) / 2.52)
    new_yaw = (data.FL - 202.8) / 2.82

    yaw_error = new_yaw - yaw_curr

    #yaw_curr = new_yaw

    read_yaw0 = True


#############################################################
def straight(t_i, pid, time_params, pwm_target):
    global yaw_error

    pwm, yaw = 0, 0
    # unpack parameters
    (t_0, t_f, dt)  = time_params

    # rest
    if t_i < t_0:
        yaw = 90
        pwm = 90

    # start moving
    elif (t_i < t_f):
        yaw = pid.update(yaw_error, dt) + 90.0
        #step_up = 90 + float(t_i - t_0) 
        #pwm = np.min([ step_up, pwm_target])
        pwm = pwm_target

    # stop experiment
    else:
        yaw = 90
        pwm = 90

    return (pwm, yaw)


def main_auto():
    global yaw_error, yaw_curr, read_yaw0, yaw_target, t_vals
    init_node('auto_mode', anonymous=True)
    ecu_pub = Publisher('ecu_pwm', ECU, queue_size = 10)
    servo_sub = Subscriber('servo', Encoder, servo_callback)

    rateHz = get_param("controller/rate")
    rate = Rate(rateHz)
    d_t = 1.0 / rateHz

    # PID params
    p = get_param("controller/p")
    i = get_param("controller/i")
    d = get_param("controller/d")

    pid = PID(P=p, I=i, D=d)

    # experiment parameters
    t_0 = get_param("controller/t_0") # start time
    t_f = get_param("controller/t_f") # end time
    time_params = (t_0, t_f, d_t)

    pwm_target = get_param("controller/pwm_target")
    yaw_target = get_param("controller/yaw_target")
    
    pid.setPoint(yaw_target)
    setReference = True
    
    t_i = 0.0
    pwm = pwm_target

    while not is_shutdown():
        if (read_yaw0):
            (pwm, yaw_curr) = straight(t_i, pid, time_params, pwm_target)

        print "pwm: ", pwm
        print "yaw_curr: ", yaw_curr

        yaw_vals.append(yaw_curr)
        t_vals.append(t_i)
        ecu_cmd = ECU(pwm, yaw_curr)
        ecu_pub.publish(ecu_cmd)
        t_i += d_t

        rate.sleep()


#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except ROSInterruptException:
		pass
