# Berkeley Autonomous Race Car (BARC)
# Last Updated 8/30/15
#
# Multithreading Program
#
#

import sys
import time
import serial
import traceback
import threading
import pyfirmata import Arduino, util

# create a PyMata instance
board = Arduino('/dev/ttyACM99')
serial_device = '/dev/ttyACM0' # USB virtual COM


# define all global variables
global_var = {
    NEUTRAL : 92
    V_X : 0
    V_Y : 0
    X_i : 0
    Y_i : 0
    CTRL_ESC : 92
    CTRL_SERVO : 92
    DELTA : 0
    YAW : 0
    YAW_RATE : 0
}

# create threading locks
imu_cont = threading.Lock()
firm_cont = threading.Lock()

########################################################
# the next group of functions are to support IMU reading
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

    # Get version
    rsp = send_command(serial_port, 'version')
    print rsp
    

    # Data transfer mode : ASCII, TRIGGER
    rsp = send_command(serial_port, 'mode,AT')
    print rsp


    # Select output message type
    rsp = send_command(serial_port, 'asc_out,RPYIMU')
    print rsp



    while (True):
    

        # send trigger command
        rsp=send_command(serial_port, 'trig')
        print rsp
        

        # wait for data message
        line = serial_port.readline().strip()
        print 'DATA MESSAGE : <%s>'%line
        

        # parse data message
        items = parse_data_message_rpyimu(line)

        # display output
        if(items):
            sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = items
            
            imu_cont.acquire(True)
            
            # Update velocities and position in global_var
            # TODO
            
            imu_cont.release();


    serial_port.close()

# End IMU reading section of code
###############################################

###############################################
# ESC, SERVO, encoder, ultrasonic function on arduino

def firm():
    encoderFL = board.digital[11]
    encoderFR = board.digital[12]
    encoderBL = board.digital[8]
    encoderBR = board.digital[9]
#    ultraFL = board.digital[1]
#    ultraFR = board.digital[2]
#    ultraBL = board.digital[6]
#    ultraBR = board.digital[7]
#
#    ultraFL._set_mode(0)
#    ultraFL.enable_reporting()
#    
#    ultraFR._set_mode(0)
#    ultraFR.enable_reporting()
#    
#    ultraBL._set_mode(0)
#    ultraBL.enable_reporting()
#    
#    ultraBR._set_mode(0)
#    ultraBR.enable_reporting()

    
    encoderFL._set_mode(0) ##INPUT = 0
    encoderFL.enable_reporting()
    encoderFL_state = encoderFL.read();
    
    encoderFR._set_mode(0)
    encoderFR.enable_reporting()
    encoderFR_state = encoderFR.read();
    
    encoderBL._set_mode(0)
    encoderBL.enable_reporting()
    encoderBL_state = encoderBL.read();
    
    encoderBR._set_mode(0)
    encoderBR.enable_reporting()
    encoderBR_state = encoderBR.read();
    
    encoderFL_count = 0
    encoderFR_count = 0
    encoderBL_count = 0
    encoderBR_count = 0
    Sample_Rate = 1 ##in seconds
    time_start = datetime.datetime.now()

    while True:
        
        updatestart = 0
    
        encoderFL_newstate = encoderFL.read()
            if (encoderFL_state != encoderFL_newstate):
                encoderFL_count += 1
                encoderFL_state = encoderFL_newstate
    
        encoderFR_newstate = encoderFR.read()
            if (encoderFR_state != encoderFR_newstate):
                encoderFR_count += 1
                encoderFR_state = encoderFR_newstate

        encoderBL_newstate = encoderBL.read()
            if (encoderBL_state != encoderBL_newstate):
                encoderBL_count += 1
                encoderBL_state = encoderBL_newstate
        
        encoderBR_newstate = encoderBR.read()
            if (encoderBR_state != encoderBR_newstate):
                encoderBR_count += 1
                encoderBR_state = encoderBR_newstate

        time_stop = datetime.datetime.now()
        
        if (time_stop.seconds - time_start.seconds > Sample_Rate):
            rpmFL = encoderFL_count*(60/Sample_Rate)/4
            rpmFR = encoderFR_count*(60/Sample_Rate)/4
            rpmBL = encoderBL_count*(60/Sample_Rate)/4
            rpmBR = encoderBR_count*(60/Sample_Rate)/4
            encoderFL_count = 0
            encoderFR_count = 0
            encoderBL_count = 0
            encoderBR_count = 0
            updatestart = 1
            
        # Calculate V_X_enc and V_Y_enc as cross check?
        # TODO
        
        # Check ultrasonics
        # TODO
        
        firm_cont.acquire(True)
        
        # update global variables as computed in this function. (Currently none)
        # TODO
        local_esc = CTRL_ESC
        local_servo = CTRL_SERVO
        
        firm_cont.release()
        
        # Write esc and servo
        signal.signal(signal.SIGINT, signal_handler)
        board.digital[5].write(local_esc)
        board.digital[3].write(local_servo)
        
        if (updatestart = 1):
                time_start = datetime.datetime.now()

# End firmata section of code
################################################

################################################
# Controller Code

def cont():

    imu_cont.acquire(True)
    firm_cont.acquire(True)

    # Controller code should have lock only for short time as to prevent blocking other threads
    local_input_var = global_var # Create copy to work with
    CTRL_ESC = comp_esc
    CTRL_SERVO = comp_servo
    
    # Controller code here

    imu_cont.release()
    firm_cont.release()

# End controller section of code
################################################


if __name__ == '__main__':

    #Set up pin modes for both pins esc and servo
    board.digital[3]._set_mode(4) # 4 is servo mode
    board.digital[5]._set_mode(4) # 4 is servo mode
    ####initialize
    board.digital[3].write(92)
    board.digital[5].write(92)
    time.sleep(2)


    thread1 = threading.Thread(target = read_example, args=(serial_device,))
    thread2 = threading.Thread(target = firm, args = ())
    thread3 = threading.Thread(target = cont, args = ())
    
    threads.append(thread1)
    thread1.start()
    threads.append(thread2)
    # car begins to run
    thread2.start()
    threads.append(thread3)
    thread3.start()






