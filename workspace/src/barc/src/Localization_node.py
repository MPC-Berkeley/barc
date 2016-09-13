#!/usr/bin/env python

# ---------------------------------------------------------------------------
#
# ---------------------------------------------------------------------------


import rospy
import time
import os
from numpy import *
from Localization_helpers import Localization
from barc.msg import Z_KinBkMdl, pos_info

l = 0
epsi_prev = 0

# State estimate callback function
def state_estimate_callback(data):
    global l, epsi_prev
	# Set current position and orientation
    l.set_pos(data.x,data.y,data.psi,data.v)
    # Update position and trajectory information
    l.find_s()

    # unwrap epsi
    l.epsi = unwrap(array([epsi_prev,l.epsi]),discont=pi)[1]
    epsi_prev = l.epsi;

# localization node
def localization_node():
    global l        # localization class

    # initialize node
    rospy.init_node('localization', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('state_estimate', Z_KinBkMdl, state_estimate_callback)
    state_pub = rospy.Publisher('pos_info', pos_info, queue_size = 10)

    # create localization class and trajectory
    l = Localization()
    # l.create_circle(1,100,array([3.2,0.5]))
    l.create_racetrack(2.0,2.0,0.2,array([0.0,-1.0]),0)
    # l.create_ellipse(1.5,0.8,100,array([2.8,1.6]))
    l.prepare_trajectory(0.063)

    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    while not rospy.is_shutdown():

        # publish information
        state_pub.publish( pos_info(l.s,l.ey,l.epsi,l.v,l.coeffX,l.coeffY,l.coeffTheta,l.coeffCurvature) )

        # wait
        rate.sleep()


if __name__ == '__main__':
    try:
       localization_node()
    except rospy.ROSInterruptException:
        pass
