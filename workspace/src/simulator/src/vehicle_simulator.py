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

import rospy
import time
from barc.msg import ECU
from simulator.msg import Z_DynBkMdl
from numpy import sin, cos, tan, arctan, array, dot, pi
from numpy import sign, argmin, sqrt, zeros, row_stack, ones, interp
from system_models_simulator import bikeFE
# input variables
d_f         = 0
acc         = 0

# from encoder
t0          = time.time()

# ecu command update
def ecu_callback(data):
    global acc, d_f
    acc         = data.motor        # input acceleration
    d_f         = data.servo        # input steering angle

# state estimation node
def vehicle_simulator():

    # initialize node
    rospy.init_node('vehicle_simulator', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('ecu', ECU, ecu_callback)
    state_pub   = rospy.Publisher('z_vhcl', Z_DynBkMdl, queue_size = 10)

    # get external force model
    a0    = rospy.get_param("air_drag_coeff")
    Ff    = rospy.get_param("friction")
    x_list     = array([0, 20, 20, 40,  40, 60, 60,  80, 80, 100, 100, 120, 120, 140, 140, 160, 160, 180])
    theta_list = array([0, 0,  10, 10,  0,  0, -10, -10,  0,  0,  30,  30,  0,   0,   -30, -30, 0,   0])
    
    # set node rate
    loop_rate   = 50
    ts          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    # set initial conditions 
    x   = 0
    y   = 0
    psi = 0
    v_x = 0
    v_y = 0
    r   = 0

    s    = 0
    ey   = 0
    epsi = 0
    while not rospy.is_shutdown():
        theta = interp(x, x_list, theta_list)/180*pi

        if x < 180:
	        (x, y, psi, v_x) = bikeFE(x, y, psi, v_x, acc, d_f, a0, Ff, theta, ts)
	        v_y = 0
	        r = 0
	        # publish information
	        state_pub.publish(Z_DynBkMdl(x, y, psi, v_x, v_y, r) )
        else:
            break
        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       vehicle_simulator()
    except rospy.ROSInterruptException:
        pass
