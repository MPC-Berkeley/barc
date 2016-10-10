#!/usr/bin/env python

# ---------------------------------------------------------------------------
#
# ---------------------------------------------------------------------------


import rospy
import time
import os
from numpy import *
from Localization_helpers import Localization
from barc.msg import Z_KinBkMdl, pos_info, Z_DynBkMdl
import matplotlib.pyplot as plt

l = 0
epsi_prev = 0
running = False

# State estimate callback function
# def state_estimate_callback(data):
#     global l, epsi_prev
#     # Set current position and orientation
#     l.set_pos(data.x,data.y,data.psi,data.v)
#     # Update position and trajectory information
#     l.find_s()
#     # unwrap epsi
#     # l.epsi = unwrap(array([epsi_prev,l.epsi]),discont=pi)[1]
#     epsi_prev = l.epsi

def dyn_state_estimate_callback(data):
    global l, epsi_prev, running
    if running:
        # Set current position and orientation
        l.set_pos(data.x,data.y,data.psi,data.v_x,data.v_x,data.v_y,data.psi_dot)        # v = v_x
        # Update position and trajectory information
        l.find_s()
        # unwrap epsi
        # l.epsi = unwrap(array([epsi_prev,l.epsi]),discont=pi)[1]
        epsi_prev = l.epsi

# localization node
def localization_node():
    global l        # localization class
    global running
    # initialize node
    rospy.init_node('localization', anonymous=True)

    # topic subscriptions / publications
    #rospy.Subscriber('state_estimate', Z_KinBkMdl, state_estimate_callback)
    rospy.Subscriber('state_estimate_dynamic', Z_DynBkMdl, dyn_state_estimate_callback)
    state_pub = rospy.Publisher('pos_info', pos_info, queue_size = 1)

    # create localization class and trajectory
    l = Localization()
    # l.create_circle(1,100,array([3.2,0.5]))
    #l.create_racetrack(2.0,2.0,0.2,array([0.0,-1.0]),0)
    l.create_track()
    #l.create_track2()
    # l.create_ellipse(1.5,0.8,100,array([2.8,1.6]))
    l.prepare_trajectory(0.06)
    #plt.plot(l.nodes[0,:],l.nodes[1,:],'-o')
    #print l.nodes
    #plt.show()

    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    while not rospy.is_shutdown():
        running = True

        # publish information
        state_pub.publish( pos_info(l.s,l.ey,l.epsi,l.v,l.s_start,l.x,l.y,l.v_x,l.v_y,l.psi,l.psiDot,l.coeffX.tolist(),l.coeffY.tolist(),l.coeffTheta.tolist(),l.coeffCurvature.tolist()) )

        # wait
        rate.sleep()


if __name__ == '__main__':
    try:
       localization_node()
    except rospy.ROSInterruptException:
        pass
