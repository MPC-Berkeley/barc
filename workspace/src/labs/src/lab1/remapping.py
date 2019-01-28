#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def callback_function(data):
    #FILL IN HERE


def subscriber_name():
    # Initialize node
    rospy.init_node('subscriber_name', anonymous=True)

    #FILL IN HERE

if __name__ == '__main__':
    try:
        subscriber_name()
    except rospy.ROSInterruptException:
        pass
