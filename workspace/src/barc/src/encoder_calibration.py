#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Author: J. Noonan
# Email:  jpnoonan@berkeley.edu
#
# This file is utilized to calibrate analog optical encoders.  Running this code 
# returns a list of threshold values to use to distinguish the changes in color 
# read by the optical sensor.  
# 
# The calibration threshold values are stored in a file named `encoder_calibration_data.txt` in the rosbag/ directory.
# ---------------------------------------------------------------------------


import rospy
import time
import os
from barc.msg import ECU, Encoder
import numpy as np

global FL_dict, FR_dict, B_dict

FL_dict = {}
FR_dict = {}
B_dict = {}

def get_threshold_values():
    global FL_dict, FR_dict, B_dict
    print B_dict
    FL_vals = FL_dict.keys()
    FL_thresh = (min(FL_vals) + max(FL_vals)) / 2.0
    FR_vals = FR_dict.keys()
    FR_thresh = (min(FR_vals) + max(FR_vals)) / 2.0
    B_vals = B_dict.keys()
    B_thresh = (min(B_vals) + max(B_vals)) / 2.0
    
    if (os.path.exists("encoder_calibration_data.txt")):
        os.remove("encoder_calibration_data.txt")
    
    with open ("encoder_calibration_data.txt", "w") as calib_file:
        calib_file.write("FL threshold: %f\n" % FL_thresh)
        calib_file.write("FR threshold: %f\n" % FR_thresh)
        calib_file.write("B  threshold: %f\n" % B_thresh)
    



def enc_s_rd_callback(msg):
    global FL_dict, FR_dict, B_dict
    try:
        FL_dict[msg.FL] += 1
    except:
        FL_dict[msg.FL] = 1
    try:
        FR_dict[msg.FR] += 1
    except:
        FR_dict[msg.FR] = 1
    try:
        B_dict[msg.BR] += 1
    except:
        B_dict[msg.BR] = 1



def main():
    # initialize node
    rospy.init_node('enc_calibration', anonymous=True)

    sub_enc_sensor_reading = rospy.Subscriber("encoder_sensor_reading", Encoder, enc_s_rd_callback)

    # set node rate
    loop_rate   = 100
    rate        = rospy.Rate(loop_rate)
    duration = rospy.get_param("enc_calibration/duration")
    start = time.time()
    while not rospy.is_shutdown():
        if ((time.time() - start) >= duration):
            get_threshold_values()
            rospy.signal_shutdown('End of encoder calibration')
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
