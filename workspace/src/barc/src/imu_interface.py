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

import traceback
from math import pi
import serial

#############################################################
def send_command(serial_port, cmd_msg):
    cmd_msg = '@' + cmd_msg.strip()
    crc = 0
    for c in cmd_msg:
        crc = crc^ord(c)
    serial_port.write(cmd_msg + '*%02X'%crc + '\r\n')

    # wait for response
    if(cmd_msg != '@trig'):
        while(True):
            line = serial_port.readline().strip()
            if(line[0] == '~'):
                return line

################################################################
g = 9.81        # acceleration due to gravity [m/s^s]
def parse_IMU_data(data):
	data = (data.split('*')[0]).strip() # discard crc field
	fields = [x.strip() for x in data.split(',')]

	if(fields[0] != '$RPYIMU'):
		return None

	_, roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z, _, _, _, _= (float(x) for x in fields[1:])

	# Use coordinate system with +x pointing forward, +y point to the left,
	# and +z point upward
	# Covert units to [rad] and [rad/s]
	roll 	*=  (pi/180)
	pitch 	*= -(pi/180)
	yaw 	*= -(pi/180)
	w_x 	*= (pi/180)
	w_y 	*= -(pi/180)
	w_z 	*= -(pi/180)
	a_x 	*= g
	a_y 	*= -g
	a_z 	*= -g
	return (roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z)

##################################################################
def IMU_initialization(serial_device):
	print 'START TEST(%s)'%(serial_device)
	try:
		serial_port = serial.Serial(serial_device, 115200, timeout=1.0)
	except serial.serialutil.SerialException:
		print 'Can not open serial port(%s)'%(serial_device)
		traceback.print_exc()
		return

	# Get version
	rsp = send_command(serial_port, 'version')
	print rsp

	# Data transfer mode : ASCII, TRIGGER
	rsp = send_command(serial_port, 'mode,AT')
	print rsp

	# Select output message type
	rsp = send_command(serial_port, 'asc_out,RPYIMU')
	print rsp

	#global file1 #file1 = open('myahrs_data.csv', 'a')
	return serial_port
