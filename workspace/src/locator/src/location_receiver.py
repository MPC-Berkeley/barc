#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform
import numpy as np

PERIOD = 2
def listener():

    rospy.init_node('receiver', anonymous=True)

    def callback(msg):
        print msg

    rospy.Subscriber("ground_true_pos", Transform, callback)


    rospy.spin()


if __name__ == '__main__':
    listener()
