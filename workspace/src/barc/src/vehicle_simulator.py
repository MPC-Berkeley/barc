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
from barc.msg import ECU, Z_KinBkMdl

# input variables [default values]
d_f         = 0         # steering angle [deg]
acc         = 0         # acceleration [m/s]

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
    state_pub   = rospy.Publisher('state_vehicle_simulator', Z_KinBkMdl, queue_size = 10)

    # get vehicle dimension parameters
    L_a = rospy.get_param("L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("L_b")       # distance from CoG to rear axel

    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    while not rospy.is_shutdown():

        # publish state estimate
        bta         = arctan( L_a / (L_a + L_b) * tan(d_f) )

        x_next      = x + dt*( v*cos(psi + bta) ) 
        y_next      = y + dt*( v*sin(psi + bta) ) 
        psi_next    = psi + dt*v/L_b*sin(bta)
        v_next      = v + dt*accc
        # publish information
        state_pub.publish( Z_KinBkMdl(x_next, y_next, psi_next, v_next) )

        x   = x_next
        y   = y_next
        v   = v_next
        psi = psi_next
        
        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       state_estimation()
    except rospy.ROSInterruptException:
        pass
