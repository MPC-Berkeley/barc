#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Authors: J. Noonan and Jon Gonzales
# Emails:  jpnoonan@berkeley.edu, jon.gonzales@berkeley.edu
#
# ---------------------------------------------------------------------------


import rospy
import time
import os
from barc.msg import ECU, Encoder, Z_KinBkMdl, Vel
from std_msgs.msg import Float32
from ekf import ekf
from system_models import f_KinBkMdl, h_KinBkMdl
from tf import transformations
import numpy as np

global servo_angle_map

servo_angle_map = {}

def servo_callback(data):
    global ecu_angle, servo_angle_map
    angle = data.FL;
    try:
        servo_angle_map[ecu_angle].append(angle)
    except:
        servo_angle_map[ecu_angle] = [angle]


def make_poly():
    global servo_angle_map
    avg_servo_angle_map = {}
    for ref_angle in servo_angle_map.keys():
        avg_servo_angle_map[ref_angle] = sum(servo_angle_map[ref_angle]) / len(servo_angle_map[ref_angle])
    
    x = np.array(avg_servo_angle_map.keys())
    y = np.array(avg_servo_angle_map.values())

    p = np.polyfit(x, y, 1)

    print "The Equation describing the relation between ECU Ref angle and Servo Steering Angle: "
    if (p[1] > 0):
        print str(p[0]) + "x + " + str(p[1])
    else:
        print str(p[0]) + "x - " + str((-p[1]))



def main():
    global ecu_angle

    rospy.init_node('servo_node', anonymous=True)
    rospy.on_shutdown(make_poly)
    ecu_pub = rospy.Publisher('ecu_pwm', ECU, queue_size=10)

    # set node rate
    loop_rate   = 50
    rate        = rospy.Rate(loop_rate)
    
    t0 = time.time()
    ecu_angle = 70

    while not rospy.is_shutdown():
        
        dt = time.time() - t0
        print "ecu_angle: " + str(ecu_angle)
        if (dt >= 1):
            ecu_angle += 1
            t0 = time.time()

        ecu_pub.publish(ECU(95, ecu_angle))
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
