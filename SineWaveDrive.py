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
from numpy import pi
from pyfirmata import Arduino, util
import threading
from IMU_sensor_model import estimate_position, imuSignal, compute_yaw_rate


global file2
global encoderFL
global encoderFR
global encoderBL
global encoderBR
global Sample_Rate
global time_curr

global serial_port
global file1

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
##def sawtooth(t,period):
##    #0<t<period, produces a sawtooth wave of amplitude 2
##    #The wave starts at neutral->positive->neutral->negative->neutral
##    if t>period or t<0:
##        return 0
##    if t<period/4:
##        print ((float) t)/(period/4)
##        return (float t)/(period/4)
##    if t> period*(3/4):
##        print ((float t)-period*(3/4))/(period/4) - 1
##        return ((float t)-period*(3/4))/(period/4) - 1
##    else:
##        print -((float t)-period/4)/(period/4) + 1
##        return -((float t)-period/4)/(period/4) + 1
##        


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
# Initialize IMU model classes.

# filter parameters
# aph   := smoothing factor, (all filtered)   0 <=   aph   <= 1  (no filter)
# n     := size of moving average block
a1          = 0.005
a2          = 0.05
n           = 200

g = 9.81
t0 = 0

Y_data      = imuSignal(y0 = [0,0,0,0], a = [a1,a1,1,a2], method = 'lp')
BF_data     = imuSignal(y0 = [0,0], n = n, method = 'mvg')
GF_data     = imuSignal(y0 = [0,0,0,0], method = None)
X_estimate  = (BF_data, GF_data)

##################################################################
def sensorModelupdate(dt,a_x,a_y,psi):
    #Updates signal class models, Y_data, BF_data, GF_data.
    a_x_new     = a_x * g 
    a_y_new     = -a_y* g
    psi_new     = -psi * (pi/180.0)
        
    psi_prev    = Y_data.getSignal('raw')[2]
    w_z_new     = compute_yaw_rate(psi_prev, psi_new, dt)

    # filter signals
    Y_data.update( [a_x_new, a_y_new, psi_new, w_z_new] )
    
    # estimate position
    estimate_position(Y_data, X_estimate, dt)

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
    t_prev = time.time()
    
    angle = NEUTRAL
    DIR = 1
    
    while True:
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
        if angle==129:
            DIR = -1
        if angle ==51:
            DIR = 1
        angle+= DIR
        board.digital[5].write(angle)
        if(items):
            sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = items
            print '## roll %.2f, pitch %.2f, yaw %.2f, ax %.4f, ay %.4f, az %.4f, gx %.4f, gy %.4f, gz %.4f, mx %.4f, my %.4f, mz %.4f, temp %.1f'%(
                roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature)
            
            time_now = time.time()
            sensorModelupdate(time_now-t_prev, accel_x, accel_y, gyro_z)
            BF_temp = BF_data.getSignal()
            GF_temp = GF_data.getSignal()
            file1.write('rpy,%.2f,%.2f,%.2f,a_xyz,%.4f,%.4f,%.4f,g_xyz,%.4f,%.4f,%.4f,t_s,%.4f, BFxy, %.4f, %.4f, GFxy, %.4f, %.4f, %d\n'%(
                roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, time.time()-tzero, BF_temp[0], BF_temp[1], GF_temp[0], GF_temp[1],angle))

            t_prev = time_now
            
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


    global Sample_Rate
    Sample_Rate = .1 ##in seconds
    
    global time_curr
    time_curr = time.time()

    
    

#################################################
def encoderThread():
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
        #print "RPM: ",rpmFL, rpmFR, rpmBL, rpmBR
        global file2
        file2.write("Time,%d,FL,%d,FR,%d,BR,%d,BL,%d"%(time_curr,rpmFL,rpmFR,rpmBL,rpmBR))




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

    

    #thread1 = KillableThread(encoderThreadInit, encoderThread)
    #threads.append(thread1)
    thread2 = KillableThread(IMUThreadInit, IMUThread)
    threads.append(thread2)

    ###############################################

    for thread in threads:
        thread.start()
    
    ##angle=NEUTRAL
    ##DIR = 1
    while (True):
        time.sleep(1)
##            if angle==129:
##                DIR = -1
##            if angle ==51:
##                DIR = 1
##            angle+= DIR
##            board.digital[5].write(angle)
##            #board.digital[5].write(NEUTRAL + 40*sawtooth(i,100))
##            time.sleep(0.01)
