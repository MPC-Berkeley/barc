#!/usr/bin/env python

import sys
from time import sleep
import serial
import struct
import rospy
import time
from numpy import pi
from geometry_msgs.msg import Vector3
from indoor_gps_interface import indoor_gps_init, get_reading
import crcmod

def indoor_gps_data_acq():
    # start node
    rospy.init_node('indoor_gps', anonymous=True)
    pub 	= rospy.Publisher('indoor_gps', Vector3, queue_size = 10)
    
    # open port
    port        = rospy.get_param("indoor_gps/port")
    baud        = rospy.get_param("indoor_gps/baud")
    ser 	    = indoor_gps_init(port, baud)

    pktSize = 23
    
    while not rospy.is_shutdown():
        ser = serial.Serial(port, baud, timeout=3 )
        ser.flushInput()
        len_buf = pktSize*3 - 1
        buf = ser.read(len_buf)
        buf = ''

        while buf is not None:
            buf += ser.read(len_buf)
            pktHdrOffset = buf.find('\xff\x47\x01\x00')
   
            while pktHdrOffset == -1:
                ser.flushInput()
                buf = ser.read(len_buf)
                pktHdrOffset = buf.find('\xff\x47\x01\x00')

            usnTimestamp, usnX, usnY, usnCRC16 = struct.unpack_from ( '<LhhxxxxxxxxH', buf, pktHdrOffset+5 )
            pub.publish( Vector3( usnX, usnY, 0) )
     
            len_tail = len( buf ) - (pktHdrOffset + pktSize)
            len_buf = pktSize - len_tail
            if len_buf == 0:
                len_buf = pktSize
                buf = ''
            elif len_buf < 0:
                ser.flushInput()
                len_buf = pktSize*2 - 1
                buf = ''
            else:
                buf = buf[ pktHdrOffset + pktSize : ]

    ser.close()

if __name__ == '__main__':
    try:
        indoor_gps_data_acq()
    except rospy.ROSInterruptException:
        pass
