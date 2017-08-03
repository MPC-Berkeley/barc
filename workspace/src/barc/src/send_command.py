#!/usr/bin/env python

import rospy
import time
import os
from sensor_msgs.msg import Imu
from barc.msg import ECU, Encoder, Z_KinBkMdl
from marvelmind_nav.msg import hedge_pos
from numpy import pi, cos, sin, eye, array, zeros, unwrap, diag
from numpy import vstack, ones, polyval, linalg, append
from ekf_CT import ekf
from system_models_CT import f_KinBkMdl, h_KinBkMdl_withGPS
from tf import transformations
from numpy import unwrap
import scipy.io as sio
import numpy as np

def send_command():
	rospy.init_node("send_command")
	loop_rate   = 50
	rate        = rospy.Rate(loop_rate)
	pub = rospy.Publisher("ecu", ECU, queue_size=10)
	while not rospy.is_shutdown():
		a_opt   = 0.3
		d_f_opt = 35*pi/180
		cmd = ECU(a_opt, d_f_opt)
		pub.publish(cmd)
		rate.sleep()

if __name__ == '__main__':
    try:
       send_command()
    except rospy.ROSInterruptException:
        pass