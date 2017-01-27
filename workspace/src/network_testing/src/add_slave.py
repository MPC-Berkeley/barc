#!/usr/bin/env python

import rospy
import socket

def add_to_slave_list():
    slave_list = rospy.get_param('slaves', [])    
    host = socket.gethostname()
    if host not in slave_list:
        slave_list.append(host)
    rospy.set_param('slaves', slave_list)



if __name__ == "__main__":
    add_to_slave_list()
