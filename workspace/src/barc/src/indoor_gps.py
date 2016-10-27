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
import serial
import time
from numpy import pi
from geometry_msgs.msg import Vector3
from indoor_gps_interface import indoor_gps_init, get_reading
import struct
import crcmod

def indoor_gps_data_acq():
    # start node
    rospy.init_node('indoor_gps', anonymous=True)
    pub 	= rospy.Publisher('indoor_gps', Vector3, queue_size = 10)
    
    # open port
    port        = rospy.get_param("indoor_gps/port")
    baud        = rospy.get_param("indoor_gps/baud")
    ser 	    = indoor_gps_init(port, baud)

    while not rospy.is_shutdown():
        data = get_reading(ser)
        
        if data:
            pub.publish( Vector3( data[0], data[1], 0) )
	
	# close connection
    ser.close()

if __name__ == '__main__':
    try:
        indoor_gps_data_acq()
    except rospy.ROSInterruptException:
        pass
