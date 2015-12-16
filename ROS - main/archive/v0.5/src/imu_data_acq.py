#!/usr/bin/python
import rospy
import serial
import time
from numpy import pi
from barc.msg import TimeData
from imu_interface import IMU_initialization, send_command, parse_IMU_data

#############################################################
def imu_data_acq():
    # launch node, publish to two topics
	rospy.init_node('imu_data_acq', anonymous=True)
	imu_data_pub 	= rospy.Publisher('imu_data', TimeData, queue_size = 10)
	smp_rate        = 50	# set publishing (sampling) rate [Hz]	
	rate            = rospy.Rate(smp_rate)

	## initialziation for IMU device
	serial_device 	= '/dev/ttyACM0'        
	serial_port 	= IMU_initialization(serial_device)

	# Collect data
	while not rospy.is_shutdown():
        # send IMU data request, wait for message, parse message
		rsp	        	= send_command(serial_port, 'trig')
		raw_data     	= serial_port.readline().strip()
		parsed_data     = parse_IMU_data(raw_data)

		# ensure imu response is not empty
		if parsed_data:
			time_now	= time.time()       # get current time

			# make a time data
			time_data 				= TimeData()
			time_data.timestamp     = time_now
			time_data.value 	    = parsed_data

            # publish data
			imu_data_pub.publish(time_data)
            
		rate.sleep()
	
	# close imu connection
	serial_port.close()

#############################################################
if __name__ == '__main__':
    try:
        imu_data_acq()
    except rospy.ROSInterruptException:
        pass
