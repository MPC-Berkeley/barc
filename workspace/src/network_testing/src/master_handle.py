#!/usr/bin/env python

import rospy
import socket



def shutdown():
    if rospy.search_param('master') is not None:
        rospy.delete_param('master')
    if rospy.search_param('slaves') is not None:
        rospy.delete_param('slaves')

def master_init():
    rospy.init_node("master_car_handler")
    if rospy.search_param('master') is None:
        host = socket.gethostname()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.set_param('master', host)
            r.sleep()
        shutdown()
    else:
        print("Something went wrong. There is already a /master parameter.")
        raise rospy.ROSInterruptException

if __name__ == "__main__":
    try:
        master_init()
    except rospy.ROSInterruptException:
        shutdown()
        pass
