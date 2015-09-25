#!/usr/bin/env python
# -*- coding: ms949 -*-
#
# python_example.py
# 2014.07.05 ('c')void
#
#
import sys
import time
import serial
import traceback
def send_command(serial_port, cmd_msg):
    cmd_msg = '@' + cmd_msg.strip()
    crc = 0
    for c in cmd_msg:
        crc = crc^ord(c)
    serial_port.write(cmd_msg + '*%02X'%crc + '\r\n')
    #
    # wait for response
    #
    if(cmd_msg != '@trig'):
        while(True):
            line = serial_port.readline().strip()
            if(line[0] == '~'):
                return line
def parse_data_message_rpyimu(data_message):
    # $RPYIMU,39,0.42,-0.31,-26.51,-0.0049,-0.0038,-1.0103,-0.0101,0.0014,-0.4001,51.9000,26.7000,11.7000,41.5*1F
    data_message = (data_message.split('*')[0]).strip() # discard crc field
    fields = [x.strip() for x in data_message.split(',')]
    if(fields[0] != '$RPYIMU'):
        return None
    sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = (float(x) for x in fields[1:])
    return (int(sequence_number), roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature)
def read_example(serial_device):
    print 'START TEST(%s)'%(serial_device)
    try:
        serial_port = serial.Serial(serial_device, 115200, timeout=1.0)
    except serial.serialutil.SerialException:
        print 'Can not open serial port(%s)'%(serial_device)
        traceback.print_exc()
        return
    #
    # Get version
    #
    rsp = send_command(serial_port, 'version')
    print rsp
    #
    # Data transfer mode : ASCII, TRIGGER
    #
    rsp = send_command(serial_port, 'mode,AT')
    print rsp
    #
    # Select output message type
    #
    rsp = send_command(serial_port, 'asc_out,RPYIMU')
    print rsp
    for i in range(1000):
        time.sleep(0.05)
        #
        # send trigger command
        #
        send_command(serial_port, 'trig')
        #
        # wait for data message
        #
        line = serial_port.readline().strip()
        #print 'DATA MESSAGE : <%s>'%line
        #
        # parse data message
        #
        items = parse_data_message_rpyimu(line)
        #
        # display output
        #
        if(items):
            sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = items
            print accel_x, accel_y, accel_z #'## roll %.2f, pitch %.2f, yaw %.2f, ax %.4f, ay %.4f, az %.4f, gx %.4f, gy %.4f, gz %.4f, mx %.4f, my %.4f, mz %.4f, temp %.1f'%(roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature)
    serial_port.close()
    print 'END OF TEST'
if __name__ == '__main__':
    if(len(sys.argv) < 2):
        serial_device = '/dev/ttyACM0'
    else :
        serial_device = sys.argv[1]
    read_example(serial_device)
