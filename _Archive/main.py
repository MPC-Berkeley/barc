# BARC main program loop
# Last updated
# 9/14/15
#

import sys
import time 
import serial
import signal
import traceback
import math
from pyfirmata import Arduino, util
import threading
import pyomo
import coopr
from coopr.pyomo import *

######################################################
#--------------Program Structure Setup---------------#
######################################################

# Structure parameters
imu_cont = threading.Lock()
enc_cont = threading.Lock()
kill_me_please_main = False
in_control = 0
global encoderFL
global encoderFR
global encoderBL
global encoderBR
global Sample_Rate
global time_curr
global serial_port
NEUTRAL = 92
# Servo is pin 5
# Esc is pin 3
tzero = time.time()
global counter
counter = 0
global imu_start

# Model Parameters
FL_speed
FR_speed
BL_speed
BR_speed
V_x_i
V_y_i
X_i
Y_i
ESC_VAL = 92 # just initially
SERVO_VAL = 92 # just initially
DELTA
YAW
YAW_RATE


class KillableThread(threading.Thread):
    def __init__(self, functionInit, function):
        threading.Thread.__init__(self)
        self.init.func = functionInit
        self.func = function
        self.kill_me_please = False
        self.args = []

    def run(self):
        self.init_func()
        while not self.kill_me_please:
            self.func()
        return

def signal_handler(sig, frame):
    if in_control == 1:
        imu_cont.release()
        enc_cont.release()
    thread1.kill_me_please = True
    thread2.kill_me_please = True
    kill_me_please_main = True
    print('Now brake')
    board.digital[5].write(NEUTRAL)
    board.digital[3].write(50)
    time.sleep(2)
    print('Now release')
    board.digital[3].write(NEUTRAL)
    board.exit()
    global serial_port
    serial_port.close()
    print('Exiting now')
    sys.exit(0)

######################################################
#----------Communication Initialization--------------#
######################################################


# create a Pyfirmata instance
board = Arduino('/dev/ttyACM99')
global serial_device
serial_device = '/dev/ttyACM0'



######################################################
#-----------------Thread Functions-------------------#
######################################################

#----------------IMU send command--------------------#
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


#----------IMU parse_data_message_rpyimu-------------#
def parse_data_message_rpyimu(data_message):
    # $RPYIMU,39,0.42,-0.31,-26.51,-0.0049,-0.0038,-1.0103,-0.0101,0.0014,-0.4001,51.9000,26.7000,11.7000,41.5*1F
    
    data_message = (data_message.split('*')[0]).strip() # discard crc field
    fields = [x.strip() for x in data_message.split(',')]
    
    if(fields[0] != '$RPYIMU'):
        return None
    
    sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = (float(x) for x in fields[1:])
    return (int(sequence_number), roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature)

#----------------IMU Initialization----------------#
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
    #print rsp
    
    #
    # Data transfer mode : ASCII, TRIGGER
    #
    rsp = send_command(serial_port, 'mode,AT')
    #print rsp

    #
    # Select output message type
    #
    rsp = send_command(serial_port, 'asc_out,RPYIMU')
    #print rsp

#----------------IMU looping thread-----------------#
def IMUThread():
    rsp=send_command(serial_port, 'trig')
    print rsp
    
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
        
        imu_curr = time.time()
        
        a_x_i = 9.81 * (math.cos(math.radians(accel_x)) - math.sin(math.radians(accel_x)))
        a_y_i = 9.81 * (math.sin(math.radians(accel_y)) + math.cos(math.radians(accel_y)))

        #compute velocities, positions, yaw and yaw rate
        if (counter == 1):
            V_x_local_i = 0
            X_i_local = 0.5 * a_x_i * math.pow((imu_curr - imu_start),2)
        else:
            V_x_i_local = V_x_i + a_x_i * (imu_curr-imu_prev)
            X_i_local = 0.5 * a_x_i * math.pow(imu_curr - imu_prev,2) + V_x_i * (imu_curr-imu_prev) + X_i
        
        if (counter == 1):
            V_y_local_i = 0
            Y_i_local = 0.5 * a_y_i * math.pow((imu_curr - imu_start),2)
        else:
            V_y_i_local = V_y_i + a_y_i * (imu_curr-imu_prev)
            Y_i_local = 0.5 * a_y_i * math.pow(imu_curr - imu_prev,2) + V_y_i * (imu_curr-imu_prev) + Y_i
        

        if (counter == 1):
            YAW_RATE_local = (yaw - YAW) / (imu_curr-imu_start)
        else:
            YAW_RATE_local = (yaw - YAW) / (imu_curr-imu_prev)

        imu_cont.acquire(True)
        global V_x_i
        global X_i
        global V_y_i
        global Y_i
        global YAW
        global YAW_RATE
        V_x_i = V_x_i_local
        X_i = X_i_local
        V_y_i = V_y_i_local
        Y_i = Y_i_local
        YAW = yaw
        YAW_RATE = YAW_RATE_local

        imu_cont.release()

        global counter
        counter += 1
        imu_prev = time.time()


#------------------Encoder Class-------------------#
class EncoderObject():
    def __init__(self, pin):
        self.pin = pin
        self.pin._set_mode(0)
        self.pin.enable_reporting()
        
        self.count = 0
        if self.pin.read is not None:
            self.state = self.pin.read()
        self.old_state = self.state
    def update(self):
        if self.pin.read is not None:
            self.state = self.pin.read()
        if (self.state != self.old_state):
            self.count = self.count + 1
            self.old_state = self.state
    def report(self):
        total = self.count
        self.count = 0
        return total


#-------------Encoder Initialization---------------#
def encoderThreadInit():
    ##Initializes all the stuff for encoders
    
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


#--------------Encoder looping thread---------------#
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

        # convert RPM to speeds
        enc_cont.acquire(True)
        global FL_speed
        global FR_speed
        global BL_speed
        global FR_speed
        FL_speed = rpmFL * 0.16666 * 0.025 * 3.14159
        FR_speed = rpmFR * 0.16666 * 0.025 * 3.14159
        BL_speed = rpmBL * 0.16666 * 0.025 * 3.14159
        FR_speed = rpmFR * 0.16666 * 0.025 * 3.14159
        enc_cont.release()

#--------------Esc/Servo initialization------------#

def ActuationInit():
    #Set up pin modes for both pins esc and servo
    board.digital[3]._set_mode(4) # 4 is servo mode
    board.digital[5]._set_mode(4) # 4 is servo mode
    ####initialize esc
    board.digital[3].write(92)
    board.digital[5].write(92)
    time.sleep(2)
    
    #####run car
    board.digital[5].write(50)
    board.digital[3].write(100)
    global DELTA
    DELTA = 0.00000488 * math.pow(SERVO_VAL,3) - 0.00247009 * math.pow(SERVO_VAL,2) - 0.22553884 * SERVO_VAL + 37.69022927
    print("Now driving")



#-------------MAIN THREAD (Controller) ------------#

signal.signal(signal.SIGINT, signal_handler)

# Initialize servo/esc
ActuationInit

# start IMU data collection
it=util.Iterator(board)
it.start()
global imu_start
imu_start = time.time()

# start threading
threads = []

thread1 = KillableThread(encoderThreadInit, encoderThread)
thread2 = KillableThread(IMUThreadInit, IMUthread)

threads.append(thread1)
threads.append(thread2)

thread1.start()
thread2.start()


while not kill_me_please_main:

    imu_cont.acquire(True)
    enc_cont.acquire(True)
    # Make copy of global variables; computation done while locks released
    
    global in_control
    global FL_speed
    global FR_speed
    global BL_speed
    global BR_speed
    global V_x_i
    global V_y_i
    global X_i
    global Y_i
    global ESC_VAL
    global SERVO_VAL
    global DELTA
    global YAW
    global YAW_RATE
    in_control = 1
    local_FL_speed = FL_speed
    local_FR_speed = FR_speed
    local_BL_speed = BL_speed
    local_BR_speed = BR_speed
    local_V_x_i = V_x_i
    local_V_y_i = V_y_i
    local_X_i = X_i
    local_Y_i = Y_i
    local_SERVO_VAL = SERVO_VAL
    local_ESC_VAL = ESC_VAL
    local_DELTA = DELTA
    local_YAW = YAW
    local_YAW_RATE = YAW_RATE

    imu_cont.release()
    enc_cont.release()
    in_control = 0
    
#    start_time = time.time();
#
#    ### Create the ipopt solver plugin using the ASL interface
#    solver = 'ipopt'
#    solver_io = 'nl'
#    stream_solver = False    # True prints solver output to screen
#    opt = SolverFactory(solver,solver_io=solver_io)
#
#    ### Create the example model
#    model = ConcreteModel()
#    model.x1 = Var(bounds=(1,5),initialize=1.0)
#    model.x2 = Var(bounds=(1,5),initialize=5.0)
#    model.x3 = Var(bounds=(1,5),initialize=5.0)
#    model.x4 = Var(bounds=(1,5),initialize=1.0)
#    model.obj = Objective(expr=model.x1*model.x4*(model.x1+model.x2+model.x3) + model.x3)
#    model.inequality = Constraint(expr=model.x1*model.x2*model.x3*model.x4 >= 25.0)
#    model.equality = Constraint(expr=model.x1**2 + model.x2**2 + model.x3**2 + model.x4**2 == 40.0)
#
#    ### Declare all suffixes
#    # Incoming Ipopt bound multipliers (obtained from solution)
#    model.ipopt_zL_out = Suffix(direction=Suffix.IMPORT)
#    model.ipopt_zU_out = Suffix(direction=Suffix.IMPORT)
#
#    # Outgoing Ipopt bound multipliers (sent to solver)
#    model.ipopt_zL_in = Suffix(direction=Suffix.EXPORT)
#    model.ipopt_zU_in = Suffix(direction=Suffix.EXPORT)
#
#    # Obtain dual solutions from first solve and send to warm start
#    model.dual = Suffix(direction=Suffix.IMPORT_EXPORT)
#
#    ### Generate the constraint expression trees if necessary
#    if solver_io != 'nl':
#        # only required when not using the ASL interface
#        model.preprocess()
#
#    ### Send the model to ipopt and collect the solution
#    print 
#    print "INITIAL SOLVE"
#
#    # solve the model, don't show any intermediate progress or output files
#    results = opt.solve(model, tee=stream_solver)
#
#    # load the results (including any values for previously declared
#    # IMPORT / IMPORT_EXPORT Suffix components)
#    model.solutions.load_from(results)
#
#    ### Print Solution
#    print "   %7s %12s %12s" % ("Value","ipopt_zL_out","ipopt_zU_out")
#    for v in [model.x1,model.x2,model.x3,model.x4]:
#        print "%s %7g %12g %12g" % ( v,
#                                     value(v),
#                                     model.ipopt_zL_out.get(v),
#                                     model.ipopt_zU_out.get(v) )
#    print "inequality.dual =", model.dual.get(model.inequality)
#    print "equality.dual   =", model.dual.get(model.equality)
#
#    ### Set Ipopt options for warm-start
#    # The current values on the ipopt_zU_out and
#    # ipopt_zL_out suffixes will be used as initial
#    # conditions for the bound multipliers to solve
#    # the new problem
#    for var in [model.x1,model.x2,model.x3,model.x4]:
#        model.ipopt_zL_in.set_value(var,model.ipopt_zL_out.get(var))
#        model.ipopt_zU_in.set_value(var,model.ipopt_zU_out.get(var))
#    opt.options['warm_start_init_point'] = 'yes'
#    opt.options['warm_start_bound_push'] = 1e-6
#    opt.options['warm_start_mult_bound_push'] = 1e-6
#    opt.options['mu_init'] = 1e-6
#
#    ###
#    ### Send the model to ipopt and collect the solution
#    print 
#    print "WARM-STARTED SOLVE"
#    results = opt.solve(model, tee=stream_solver)
#    # load the results (including any values for previously declared
#    # IMPORT / IMPORT_EXPORT Suffix components)
#    model.solutions.load_from(results)
#
#    ###
#    ### Print Solution
#    print "   %7s %12s %12s" % ("Value","ipopt_zL_out","ipopt_zU_out")
#    for v in [model.x1,model.x2,model.x3,model.x4]:
#        print "%s %7g %12g %12g" % ( v,
#                                     value(v),
#                                     model.ipopt_zL_out.get(v),
#                                     model.ipopt_zU_out.get(v) )
#    print "inequality.dual =", model.dual.get(model.inequality)
#    print "equality.dual   =", model.dual.get(model.equality)
#
#    print("--- %s seconds ---" % (time.time() - start_time))

    global DELTA
    global SERVO_VAL
    DELTA = comp_DELTA
    SERVO_VAL =  168.722 + 3.0103 * math.pow(10,-6) * math.pow((5071.44 * math.pow(5.48507*math.pow(10,35)*math.pow(comp_DELTA,2) + 5.1837*math.pow(10,37)*comp_DELTA - 3.18822*math.pow(10,39),0.5) + 3.75598*math.pow(10,21)*comp_DELTA + 1.77435*math.pow(10,23),0.6666) + 1.45742*math.pow(10,10)/(math.pow((5071.44 * math.pow(5.48507*math.pow(10,35)*math.pow(comp_DELTA,2) + 5.1837*math.pow(10,37)*comp_DELTA - 3.18822*math.pow(10,39),0.5) + 3.75598*math.pow(10,21)*comp_DELTA + 1.77435*math.pow(10,23),0.6666))
    SERVO_VAL = board.digital[5].write(SERVO_VAL)







    

    
           

           
    
  

    

