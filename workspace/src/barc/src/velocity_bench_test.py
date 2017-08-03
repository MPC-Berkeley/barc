#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
# 
# Authors: J. Noonan and Jon Gonzales
# Emails:  jpnoonan@berkeley.edu, jon.gonzales@berkeley.edu
# ---------------------------------------------------------------------------

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException
from barc.msg import ECU
from numpy import pi
import rospy

def main_auto():
    # initialize ROS node
    init_node('auto_mode', anonymous=True)
    nh = Publisher('ecu', ECU, queue_size = 10)

	# set node rate
    rateHz  = 50
    rate 	= Rate(rateHz)
    dt   	= 1.0 / rateHz
    t_i     = 0

    # get experiment parameters 
    t_0 = 1
    t_f = 61
    FxR = 0.5
    d_f = pi/180*25
    # main loop
    while not is_shutdown():
        ecu_cmd = ECU(FxR, d_f)
        nh.publish(ecu_cmd)
        FxR += 0.005
        print "FxR: ", str(FxR)
        
        # wait
        t_i += dt
        rate.sleep()

#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except ROSInterruptException:
		pass
