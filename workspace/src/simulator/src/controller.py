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
from simulator.msg import Z_DynBkMdl, Z_DynBkMdlErrorFrame 

# from encoder
x        = 0
y        = 0
s        = 0
ey       = 0 
epsi     = 0                     # counts in the front left tire
psi_dot  = 0                     # counts in the front right tire
v_x      = 0
v_y      = 0

# ecu command update
def measurements_callback(data):
    global x, y, psi, v_x, v_y, psi_dot
    x        = data.x          # input acceleration
    y        = data.y          # input steering angle
    psi      = data.psi        # input acceleration
    v_x      = data.v_x        # input acceleration
    v_y      = data.v_y        # input steering angle
    psi_dot  = data.psi_dot    # input steering angle
    
def measurements_error_frame_callback(data):
    global s, ey, epsi, v_x, v_y, psi_dot
    s        = data.s          # input acceleration
    ey       = data.ey          # input steering angle
    epsi     = data.epsi        # input acceleration
    v_x      = data.v_x        # input acceleration
    v_y      = data.v_y        # input steering angle
    psi_dot  = data.psi_dot    # input steering angle

# state estimation node
def controller():

    # initialize node
    rospy.init_node('vehicle_simulator', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('state_vehicle_simulator', Z_DynBkMdl, measurements_callback)
    rospy.Subscriber('state_vehicle_error_frame_simulator', Z_DynBkMdlErrorFrame, measurements_error_frame_callback)

    state_pub   = rospy.Publisher('ecu', ECU, queue_size = 10)

    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    # set initial conditions 
    d_f = 0
    acc = 0

    while not rospy.is_shutdown():

        # publish state estimate

        acc = (1 - v_x)
        d_f = 0.1*(0 - ey) + (0 - epsi)

        # publish information
        state_pub.publish( ECU(acc, d_f) )

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       controller()
    except rospy.ROSInterruptException:
        pass
