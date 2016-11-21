#!/usr/bin/env python

# Adapted from code written by Boris Zinin (b.zinin@gmail.com)
import rospy
import serial
import crcmod
import struct
import traceback

def indoor_gps_init(serial_device, baudrate):
    # open port
    try:
        ser = serial.Serial(serial_device, baudrate, timeout=3 )
    except serial.serialutil.SerialException:
        print 'Can not open serial port(%s)'%(serial_device)
        traceback.print_exc()
        return None
 
    # return serial port
    ser.flushInput()
    return ser

def get_reading(ser):
    # read buffer chunck
    pktSize = 23
    len_buf = pktSize*2 - 1
    buf = ser.read(len_buf)
    pktHdrOffset = buf.find('\xff\x47\x01\x00')
   
    # keep reading until good packet found
    while pktHdrOffset == -1 or pktHdrOffset > 8:
        ser.flushInput()
        buf = ser.read(len_buf)
        pktHdrOffset = buf.find('\xff\x47\x01\x00')

    # parse the packet
    usnTimestamp, usnX, usnY, usnCRC16 = struct.unpack_from ( '<LhhxxxxxxxxH', buf, pktHdrOffset+5 )

    # check packet CRC
    crc16 = crcmod.predefined.Crc('modbus')
    crc16.update( buf[ pktHdrOffset : pktHdrOffset + pktSize - 2 ] )
    CRC_calc = int( crc16.hexdigest(), 16 )

    if CRC_calc == usnCRC16:
        return (usnX, usnY) 
    else:
        return None
