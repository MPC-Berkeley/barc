#!/usr/bin/env python
# -*- coding: ms949 -*-
#
#     python_example.py 
#     2014.07.05 ('c')void 
#
#

import sys
import time 
import serial 
import signal
import traceback
from pyfirmata import Arduino, util
import threading
import PID


global file2
global encoderFL
global encoderFR
global encoderBL
global encoderBR
global Sample_Rate
global time_curr

Sample_Rate = .3 ##in seconds

global serial_port
global file1

global RPM_FL_global
global RPM_FR_global
global RPM_BL_global
global RPM_BR_global

RPM_FL_global = 0
RPM_FR_global = 0
RPM_BL_global = 0
RPM_BR_global = 0

global rpmFL
global rpmFR
global rpmBL
global rpmBR


rpmFL =0
rpmFR =0
rpmBL =0
rpmBR =0

# create a PyMata instance
board = Arduino('/dev/ttyACM99')
global serial_device
serial_device = '/dev/ttyACM0'
ESC1 = 3
SERVO1 = 5
NEUTRAL = 92
tzero = time.time()
escval = 0
servoval = 0
PID_controller = PID.PID(P=0.1, I=0.0001, D=0.0003)
PID_controller.setPoint(1000)
max_throttle = 140
min_throttle = 93

##########################################################
class KillableThread(threading.Thread):
    def __init__(self, functionInit, function):
        threading.Thread.__init__(self)
        self.init_func = functionInit
        self.func = function
        self.kill_me_please = False
        self.args = []

    def run(self):
        self.init_func()
        while not self.kill_me_please:
            self.func()
        return



#############################################################
class EncoderObject():
    def __init__(self, pin):
        self.pin = pin
        self.pin._set_mode(0)
        self.pin.enable_reporting()
        
        self.count = 0
        self.state = self.pin.read()
        self.old_state = self.state
    def update(self):
        self.state = self.pin.read()
        if (self.state != self.old_state):
            self.count = self.count + 1
            self.old_state = self.state
    def report(self):
        total = self.count
        self.count = 0
        return total
    


#############################################################
def sawtooth(t,period):
    #0<t<period, produces a sawtooth wave of amplitude 2
    #The wave starts at neutral->positive->neutral->negative->neutral
    if t>period or t<0:
        return 0
    if t<period/4:
        return t/(period/4)
    if t> period*(3/4):
        return (t-period*(3/4))/(period/4) - 1
    else:
        return -(t-period/4)/(period/4) + 1
        


#############################################################
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

################################################################
def parse_data_message_rpyimu(data_message):
    # $RPYIMU,39,0.42,-0.31,-26.51,-0.0049,-0.0038,-1.0103,-0.0101,0.0014,-0.4001,51.9000,26.7000,11.7000,41.5*1F
    
    data_message = (data_message.split('*')[0]).strip() # discard crc field  
    fields = [x.strip() for x in data_message.split(',')]
    
    if(fields[0] != '$RPYIMU'):
        return None
    
    sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = (float(x) for x in fields[1:])
    return (int(sequence_number), roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature)
    
##################################################################
def IMUThreadInit():
    global serial_device
    print 'START TEST(%s)'%(serial_device)
    global serial_port
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

    global file1
    file1 = open('myahrs_data.csv', 'a')





###################################################################
def IMUThread():
    rsp=send_command(serial_port, 'trig')
    print rsp
    
    #
    # wait for data message 
    #
    line = serial_port.readline().strip()
    print 'DATA MESSAGE : <%s>'%line
    
    #
    # parse data message
    #
    items = parse_data_message_rpyimu(line)
    #
    # display output 
    #
    if(items):
        sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = items
        #print '## roll %.2f, pitch %.2f, yaw %.2f, ax %.4f, ay %.4f, az %.4f, gx %.4f, gy %.4f, gz %.4f, mx %.4f, my %.4f, mz %.4f, temp %.1f'%(roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature)
        
        file1.write('rpy,%.2f,%.2f,%.2f,a_xyz,%.4f,%.4f,%.4f,g_xyz%.4f,%.4f,%.4f,t_s,%.4f\n'%(roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, time.time()-tzero))
        
####################################################################

def encoderThreadInit():
    #########################################
    ##Initializes all the stuff for encoders
    

    global file2
    file2 = open('rpm_data.csv', 'a')

    global encoderFL
    encoderFL = EncoderObject(board.digital[11])

    global encoderFR
    encoderFR = EncoderObject(board.digital[12])

    global encoderBL
    encoderBL = EncoderObject(board.digital[8])

    global encoderBR
    encoderBR = EncoderObject(board.digital[9])


    
    
    global time_curr
    time_curr = time.time()

    
    

#################################################
def encoderThread():
    global rpmFL, rpmFR, rpmBL, rpmBR
    global encoderFL
    encoderFL.update()
    
    global encoderFR
    encoderFR.update()

    global encoderBL
    encoderBL.update()

    global encoderBR
    encoderBR.update()

    global time_curr
    new_time = time.time()
    if ((new_time - time_curr) > Sample_Rate):
        time_curr = new_time
        rpmFL = encoderFL.report()*(60/Sample_Rate)/4
        rpmFR = encoderFR.report()*(60/Sample_Rate)/4
        rpmBL = encoderBL.report()*(60/Sample_Rate)/4
        rpmBR = encoderBR.report()*(60/Sample_Rate)/4
        print 'RPM:%d,%d;\n'%(rpmBL, rpmBR)
        # global file2
        # file2.write("Time,%d,FL,%d,FR,%d,BR,%d,BL,%d"%(time_curr,rpmFL,rpmFR,rpmBL,rpmBR))
    global RPM_FL_global, RPM_BL_global, RPM_FR_global, RPM_BR_global
    RPM_FL_global = rpmFL
    RPM_FR_global = rpmFR
    RPM_BL_global = rpmBL
    RPM_BR_global = rpmBR

#################################################################
def PIDThreadInit():
    return

#################################################################
def PIDThread():
    global PID_controller
    global Sample_Rate
    global RPM_FL_global, RPM_FR_global, RPM_BL_global, RPM_BR_global
    global max_throttle, min_throttle
    FL_speed = RPM_FL_global
    FR_speed = RPM_FR_global
    BL_speed = RPM_BL_global
    BR_speed = RPM_BR_global
    current_speed = (BL_speed + BR_speed)*0.50
    next_ESC = PID_controller.update(current_speed, Sample_Rate)
    print "Error is: ", PID_controller.getError(), "\n"
    if (next_ESC >= max_throttle):
        next_ESC = max_throttle
    elif (next_ESC <= min_throttle):
        next_ESC = min_throttle
    print "next ESC is ",next_ESC,"\n"
    board.digital[ESC1].write(next_ESC)
    


###############################################################3
# Control-C signal handler to suppress exceptions if user presses Crtl+C
def signal_handler(sig, frame):
    for thread in threads:
        thread.kill_me_please = True    
    print('Now brake')
    board.digital[5].write(NEUTRAL)
    board.digital[3].write(50)
    time.sleep(2)
    print('Now release')
    board.digital[3].write(NEUTRAL)
    board.exit()
    global serial_port
    serial_port.close()

    global file1
    file1.close()
    global file2
    file2.close()
    #serial_port.close()    
    sys.exit(0)

    
#######################################################################
if __name__ == '__main__':
    #if(len(sys.argv) == 3):
    escval = NEUTRAL
    servoval = NEUTRAL
##    else:
##        print 'wrong number of args'
##        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    threads = []
    #Set up pin modes for both pins esc and servo
    board.digital[3]._set_mode(4) # 4 is servo mode
    board.digital[5]._set_mode(4) # 4 is servo mode
    ####initialize esc
    board.digital[3].write(NEUTRAL)
    board.digital[5].write(NEUTRAL)
    print 'arming the ESC'
    time.sleep(2)

    #####run car
    board.digital[5].write(NEUTRAL)
    board.digital[3].write(95)
    print("Now driving")
    # start IMU data collection
    it = util.Iterator(board)
    it.start()

    

    thread1 = KillableThread(encoderThreadInit, encoderThread)
    threads.append(thread1)
    # thread2 = KillableThread(IMUThreadInit, IMUThread)
    # threads.append(thread2)
    thread3 = KillableThread(PIDThreadInit, PIDThread)
    threads.append(thread3)

    ###############################################

    for thread in threads:
        thread.start()
    
    while (True):
        time.sleep(1)
        # for i in range(0,100):
            # board.digital[5].write(NEUTRAL + 60*sawtooth(i,100))
            # time.sleep(0.01)
