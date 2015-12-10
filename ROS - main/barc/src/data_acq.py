#!/usr/bin/python
import rospy
import serial
import time
from numpy import pi
from data_service.msg import *
from data_service.srv import *
#from data_connection import *
from geometry_msgs.msg import Vector3
from obs_mdl import imuSignal, estimation_update
from IMU_interface import IMU_initialization, send_command, parse_IMU_data

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

#############################################################
def data_acq():
        # launch node, publish to two topics
	rospy.init_node('data_acq', anonymous=True)
	imu_data_pub 	= rospy.Publisher('imu_data', TimeData, queue_size = 10)
	obs_pub 	= rospy.Publisher('state_estimate', Vector3, queue_size = 10)
	smp_rate        = 50	# set publishing (sampling) rate [Hz]	
	rate            = rospy.Rate(smp_rate)

	## initialziation for IMU device
        serial_device 	= '/dev/ttyACM0'        
        #serial_device 	= '/dev/ttyACM99'        
	serial_port 	= IMU_initialization(serial_device)

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
            time_now	= time.time()       # get current time
            items       = parse_IMU_data(line)

            if items and not t0:
                # update state estimates
                (a_x, a_y, psi, w_z)    = (items[3], items[4], items[2], items[8])
                dt  			= time_now-prev_t
                estimation_update(Y_data, X_estimate, dt, a_x, a_y, psi, w_z)

                (v_x, v_y)		= v_hat_BF.getSignal()
                r                       = Y_data.getSignal()[3]
                (Xest, Yest, _, _)      = X_hat_GF.getSignal()

                # make a time data
                items = items +(v_x,v_y,Xest,Yest)
                time_data 			= TimeData()
                time_data.timestamp             = time_now
                time_data.value 	        = items

                # log and publish data to topics
                rospy.loginfo(time_data)
                imu_data_pub.publish(time_data)
                obs_pub.publish( Vector3(v_x, v_y, r) )
            
            t0      = False
            prev_t  = time_now
            rate.sleep()

	serial_port.close()

#############################################################
if __name__ == '__main__':
    try:
        data_acq()
    except rospy.ROSInterruptException:
        pass
