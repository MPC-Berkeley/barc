#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Author: Lukas Brunke
# Email: lukas.brunke@tuhh.de
#
# ---------------------------------------------------------------------------

import time
import rospy
from barc.msg import prediction


def callback(test):
    rospy.loginfo(test)


def receiving_prediction():
    rospy.init_node("connection_test", anonymous=True)
    rospy.Subscriber("prediction", prediction, callback)
    
    while not rospy.is_shutdown():
    	rospy.spin()


if __name__ == '__main__':
    try:
        receiving_prediction()
    except rospy.ROSInterruptException:
        pass
