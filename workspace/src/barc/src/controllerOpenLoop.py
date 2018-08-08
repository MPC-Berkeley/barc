#!/usr/bin/env python
"""
    File name: controllerOpenLoop.py
    Author: Shuqi Xu
    Email: shuqixu@berkeley.edu (xushuqi8787@gmail.com)
    Python Version: 2.7.12
"""
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
from barc.msg import ECU, pos_info

class Controller(object):
    def __init__(self,t0,loop_rate):
        self.t0     = t0
        self.rate   = rospy.Rate(loop_rate)
        self.ecuPub = rospy.Publisher('ecu', ECU, queue_size=1)
        # input publishing
        self.a      = 0.4
        self.df     = 0.0

if __name__ == '__main__':
    try:
        rospy.init_node("controller")
        t0 = rospy.get_rostime().to_sec()
        controller = Controller(t0,20)
        msg = ECU()
        msg.motor = controller.a
        msg.servo = controller.df
        while not rospy.is_shutdown():
            controller.ecuPub.publish(msg)
            controller.rate.sleep()
    except rospy.ROSInterruptException:
        pass