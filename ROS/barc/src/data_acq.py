#!/usr/bin/python
import rospy
import serial
import time
from numpy import pi
from data_service.msg import *
from data_service.srv import *
from data_connection import *
from geometry_msgs.msg import Vector3
from IMU_sensor_model import estimate_position, imuSignal

# IMU sensor port
serial_device 	= '/dev/ttyACM0'        

##################################################################
def sensorModelupdate(Y_data, X_estimate, dt, a_x, a_y, psi, w_z):
    # Updates signal class models, Y_data, BF_data, GF_data.
    g = 9.81
    a_x_new     = a_x * g
    a_y_new     = -a_y* g
    psi_new     = -psi * (pi/180.0)
    w_z_new     = -w_z * (pi/180.0)

    # filter signals
    Y_data.update( [a_x_new, a_y_new, psi_new, w_z_new] )

    # estimate position
    estimate_position(Y_data, X_estimate, dt)

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
def parse_data_message_rpyimu(data_message):
	# $RPYIMU,39,0.42,-0.31,-26.51,-0.0049,-0.0038,-1.0103,-0.0101,0.0014,-0.4001,51.9000,26.7000,11.7000,41.5*1F
	data_message = (data_message.split('*')[0]).strip() # discard crc field
	fields = [x.strip() for x in data_message.split(',')]

	if(fields[0] != '$RPYIMU'):
		return None

	seq_num, roll, pitch, yaw, a_x, a_y, a_z, g_x, g_y, g_z, mag_x, mag_y, mag_z, temp = (float(x) for x in fields[1:])
	return (int(seq_num), roll, pitch, yaw, a_x, a_y, a_z, g_x, g_y, g_z, mag_x, mag_y, mag_z, temp)

################################################################
def file_store(data_imu):
     file1 = open('myahrs_data.csv', 'a')
     for data_at_one_step in data_imu.series:
       sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = data_at_one_step.value
       t = data_at_one_step.timestamp
       file1.write('rpy,%.2f,%.2f,%.2f,a_xyz,%.4f,%.4f,%.4f,g_xyz,%.4f,%.4f,%.4f,t_s,%.4f,\n'%(
                roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, t))
     return

################################################################
def cloud_store(data_imu):
    #do not like this serialize/deserialize
    signals_vector_id=['roll', 'pitch', 'yaw', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']
    lenD=9
    tsvec=[]
    signals_vector_data=[[]  for i in range(lenD)]
    for data_at_one_step in data_imu.series:
        sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = data_at_one_step.value
        signals_vector_data[0].append(roll)
        signals_vector_data[1].append(roll)
        signals_vector_data[2].append(roll)
        signals_vector_data[3].append(roll)
        signals_vector_data[4].append(roll)
        signals_vector_data[5].append(roll)
        signals_vector_data[6].append(roll)
        signals_vector_data[7].append(roll)
        signals_vector_data[8].append(roll)
        tsvec.append(DataConnection.utc_to_millisec(data_at_one_step.timestamp))
    rospy.wait_for_service('send_data')
    for i in range(lenD):
		try:
			 send_data = rospy.ServiceProxy('send_data', DataForward)
			 time_signal = TimeSignal
			 time_signal.id = signals_vector_id[i]
			 time_signal.timestamps = tsvec
			 time_signal.signal = signals_vector_data[i]
			 response = send_data(time_signal, None)
			 print 'Done'
		except rospy.ServiceException, e:
			 print 'Call to service failed: %s' %e
    return

##################################################################
def IMUThreadInit(serial_device):
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


#############################################################
def data_acq():
	# launch node, publish to two topics
	rospy.init_node('data_acq', anonymous=True)
	imu_data_pub 	= rospy.Publisher('imu_data', TimeData, queue_size = 10)
	obs_pub 	= rospy.Publisher('state_estimate', Vector3, queue_size = 10)
	smp_rate        = 20	# set publishing (sampling) rate [Hz]	
	rate            = rospy.Rate(smp_rate)

	## initialziation for IMU device
	serial_port 	= IMUThreadInit(serial_device)
	stored_k 	= 0
	stored_N 	= 20
	stored_items 	= TimeSeries()

	# filter parameters
	# p   := smoothing factor, (all filtered)   0 <=   aph   <= 1  (no filter)
	# n   := size of moving average block
	p_ax        = 0.02          # filter parameter for a_x
	p_ay        = p_ax          # filter parameter for a_y
	p_wz        = 0.9           # filter parameter for w_z
        n_v         = 100           # moving average filter parameter for v_x, v_y

	# Initialize estimation of states
	# Y_data 	:= object to hold filtered sensor data
	# v_hat_BF 	:= estimate of velocity in body frame
	# X_hat_GF 	:= estimate of position/velocity in global frame
	Y_data      = imuSignal(y0 = [0,0,0,0], a = [p_ax, p_ay, 1, p_wz], method = 'lp')
	v_hat_BF    = imuSignal(y0 = [0,0], n = n_v, method = 'mvg')
	X_hat_GF    = imuSignal(y0 = [0,0,0,0], method = None)
	X_estimate  = (v_hat_BF, X_hat_GF)
        
        t0 = True               # initial time

	# Collect data
	while not rospy.is_shutdown():
            # send IMU data request, wait for message, parse message
            rsp	        = send_command(serial_port, 'trig')
            line        = serial_port.readline().strip()
            items       = parse_data_message_rpyimu(line)

            time_now	= time.time()       # get current time

            if items and not t0:
                # update state estimates
                (a_x, a_y, psi, w_z) = (items[4], items[5], items[3], items[9])
                dt  			= time.time() - time_now
                sensorModelupdate(Y_data, X_estimate, dt, a_x, a_y, psi, w_z)
                (v_x, v_y)		= v_hat_BF.getSignal()

                # make a time data
                time_data 			= TimeData()
                time_data.timestamp             = time_now
                time_data.value 	        = items

                # log and publish data to topics
                rospy.loginfo(time_data)
                imu_data_pub.publish(time_data)
                obs_pub.publish( Vector3(v_x, v_y, r) )

                # Store data in batch
                stored_items.series.append(time_data)
                stored_k 	= stored_k + 1
                if stored_k == stored_N:
                        print 'Storing IMU Data ....\n'

                        # File storing 
                        file_store(stored_items)
                        stored_k = 0
                        stored_items.series = []

                        # Cloud Storing # cloud_store(stored_items)
            
            t0 = False
            rate.sleep()

	serial_port.close()

#############################################################
if __name__ == '__main__':
    try:
        data_acq()
    except rospy.ROSInterruptException:
        pass
