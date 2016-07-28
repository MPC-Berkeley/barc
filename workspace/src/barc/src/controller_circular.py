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

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException
from barc.msg import ECU
from numpy import pi
import rospy

#############################################################
def circular(t_i, t_0, t_f, d_f_target, FxR_target):
    # rest
    if t_i < t_0:
        d_f     = 0
        FxR     = 0

    # start moving
    elif (t_i < t_f):
        d_f     = d_f_target
        FxR     = FxR_target

    # stop experiment
    else:
        d_f     = 0
        FxR     = 0

    return (FxR, d_f)


#############################################################
def main_auto():
    # initialize ROS node
    init_node('auto_mode', anonymous=True)
    nh = Publisher('ecu', ECU, queue_size = 10)

	# set node rate
    rateHz  = get_param("controller/rate")
    rate 	= Rate(rateHz)
    dt   	= 1.0 / rateHz
    t_i     = 0

    # get experiment parameters 
    t_0             = get_param("controller/t_0")     # time to start test
    t_f             = get_param("controller/t_f")     # time to end test
    FxR_target      = get_param("controller/FxR_target")
    d_f_target      = pi/180*get_param("controller/d_f_target")
 
    # main loop
    while not is_shutdown():
        # get command signal
        (FxR, d_f) = circular(t_i, t_0, t_f, d_f_target, FxR_target)
			
        # send command signal 
        ecu_cmd = ECU(FxR, d_f)
        nh.publish(ecu_cmd)
	
        # wait
        t_i += dt
        rate.sleep()

#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except ROSInterruptException:
		pass
