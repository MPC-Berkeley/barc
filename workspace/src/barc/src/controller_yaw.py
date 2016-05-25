#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

# This controller can be launched with cruiseControl.launch
# Requires a kinematic state estimator node and an arduino hub node

import rospy
from barc.msg import ECU
from data_service.msg import TimeData
from numpy import unwrap, array
from input_map import angle_2_servo
from pid import PID

# pid control for constant yaw angle
yaw0 = 0
read_yaw0 = False
yaw_prev = 0
yaw = 0
err = 0
def imu_callback(data):
    global yaw0, read_yaw0, yaw_prev, yaw, err

    # eextract yaw angle
    (_,_,yaw, _,_,_, _,_,_) = data.value

    #save initial measurements
    if not read_yaw0:
        read_yaw0 = True
        yaw0 = yaw
    else:
        temp        = unwrap(array([yaw_prev, yaw]))
        yaw         = temp[1]
        yaw_prev    = yaw

    err = yaw - yaw0

#############################################################
def control_yaw():

    rospy.init_node('controller_yaw')

    steering_pub = rospy.Publisher('servo_pwm', ECU, queue_size = 10)
    rospy.Subscriber('imu', TimeData, imu_callback)

    # set node rate
    rateHz  = 50
    dt      = 1.0 / rateHz
    rate    = rospy.Rate(rateHz)

    str_ang = rospy.get_param("controller_yaw/steering_angle")

    # use pid to correct yaw angle, begins with set point at initial yaw angle
    p       = rospy.get_param("controller_yaw/p")
    i       = rospy.get_param("controller_yaw/i")
    d       = rospy.get_param("controller_yaw/d")
    pid     = PID(P=p, I=i, D=d)

    # main loop
    while not rospy.is_shutdown():
        # get steering wheel command
        u = pid.update(err, dt)
        servoCMD = angle_2_servo(u)

        # send command signal
        ecu_cmd = ECU(0, servoCMD)
        steering_pub.publish(ecu_cmd)

        #wait
        rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        control_yaw()
    except rospy.ROSInterruptException:
        pass
