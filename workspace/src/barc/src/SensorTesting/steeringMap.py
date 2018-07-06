#!/usr/bin/env python

import thread
import sys
from sensor_msgs.msg import Imu
import rospy
from marvelmind_nav.msg import hedge_pos, hedge_imu_fusion
import numpy as np
from barc.msg import pos_info, prediction, SafeSetGlob, Vel_est
from tf import transformations
import matplotlib.pyplot as plt    
import pdb
import matplotlib.patches as patches
import numpy as np
from numpy import sin, cos
import pickle
from barc.msg import pos_info, ECU, prediction, SafeSetGlob


def input_thread(a_list):
	input = raw_input()
	a_list.append(input)

def main():
	rospy.init_node("steeringMap")

	input_commands = rospy.Publisher('ecu_pwm', ECU, queue_size = 1)


	t0 = rospy.get_rostime().to_sec()
	imu = ImuClass(t0)
	enc = EncClass(t0)
	cmd = ECU()

	loop_rate = 100
	rate = rospy.Rate(loop_rate)

	steering_his = []
	PWMsteering_his = []
	vx_his = []

	while True!=False:
		okFlag = False
		vx_his = []

		while okFlag==False:
			print "Enter PWM for Steering"
			PWM_Steering = raw_input()
			print "Enter PWM for Acceleration"
			PWM_Input = raw_input()
			print "Steering you selected", PWM_Steering, ". Acceleration you selected", PWM_Input, ". Now enter ok to continue the test or q to exit"
			confirmInput = raw_input()
			okFlag = confirmInput == "ok"
			if confirmInput == "q":
				# Save Data
				file_data = open(sys.path[0]+'/../data/sensorTesting/steeringMap_BARC'+'.obj', 'wb')
				pickle.dump(PWMsteering_his, file_data)
				pickle.dump(steering_his, file_data)
				file_data.close()
	
		a_list = [0]
		thread.start_new_thread(input_thread, (a_list,))
		while a_list[-1] != "s":
			cmd.servo = float(PWM_Steering)
			cmd.motor = float(PWM_Input)
			input_commands.publish(cmd)
			vx_his.append(enc.v_meas)

			if enc.v_meas>0.9:
				steering = np.arctan(imu.psiDot*(0.25)/enc.v_meas)
				steering_his.append(steering)
				PWMsteering_his.append(PWM_Steering)

			rate.sleep()

		cmd.servo = 90.0
		cmd.motor = 90.0
		input_commands.publish(cmd)

		if steering_his != []:
			# Plotting
			plt.figure(1)
			plt.plot(vx_his, '-o')
			plt.legend()
			plt.ylabel('vx_his')

			ax = plt.figure(2)
			plt.plot(PWMsteering_his, steering_his, 'o')
			plt.legend()
			plt.ylabel('steering')
			plt.xlabel('PWM')
			plt.xlim((-150, 150))

			# Compute Values
			A = np.ones((len(steering_his),2))
			y = np.zeros(len(PWMsteering_his))

			Elements_pos = len([1 for i in steering_his if i >= 0])
			A_pos = np.ones((Elements_pos,2))
			y_pos = np.zeros(Elements_pos)

			Elements_neg = len([1 for i in steering_his if i <= 0])
			A_neg = np.ones((Elements_neg,2))
			y_neg = np.zeros(Elements_neg)

			counter_pos = 0
			counter_neg = 0
			for i in range(0, len(PWMsteering_his)):
				A[i,0] = PWMsteering_his[i]
				y[i]   = steering_his[i]
				
				if steering_his[i] >= 0:
					A_pos[counter_pos,0] = PWMsteering_his[i]
					y_pos[counter_pos]   = steering_his[i]
					counter_pos = counter_pos + 1

				if steering_his[i] <= 0:
					A_neg[counter_neg,0] = PWMsteering_his[i]
					y_neg[counter_neg]   = steering_his[i]
					counter_neg = counter_neg + 1

			m, c = np.linalg.lstsq(A, y)[0]
			print "The fitted m is ", m, " and the fitted c is ", c

			plt.plot(PWMsteering_his, [m*float(a)+c for a in PWMsteering_his], '-r*')

			if Elements_pos>0:
				m_pos, c_pos = np.linalg.lstsq(A_pos, y_pos)[0]
				print "The positive fitted m is ", m_pos, " and the fitted c is ", c_pos
				plt.plot(PWMsteering_his, [m_pos*float(a)+c_pos for a in PWMsteering_his], '-bs')

			if Elements_neg>0:
				m_neg, c_neg = np.linalg.lstsq(A_neg, y_neg)[0]
				print "The negative fitted m is ", m_neg, " and the fitted c is ", c_neg
				plt.plot(PWMsteering_his, [m_neg*float(a)+c_neg for a in PWMsteering_his], '-ko')


			plt.show()	



# ================================================================================================================== #
# ================================================================================================================== #
# ================================================================================================================== #
# ================================================================================================================== #
class EncClass(object):
    """ Object collecting ENC measurement data
    Attributes:
        Measurement:
            1.v_fl 2.v_fr 3. v_rl 4. v_rr
        Measurement history:
            1.v_fl_his 2.v_fr_his 3. v_rl_his 4. v_rr_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        rospy.Subscriber('vel_est', Vel_est, self.enc_callback, queue_size=1)

        # ENC measurement
        self.v_fl      = 0.0
        self.v_fr      = 0.0
        self.v_rl      = 0.0
        self.v_rr      = 0.0
        self.v_meas    = 0.0
        
        # ENC measurement history
        self.v_fl_his    = []
        self.v_fr_his    = []
        self.v_rl_his    = []
        self.v_rr_his    = []
        self.v_meas_his  = []
        
        # time stamp
        self.v_count    = 0
        self.v_prev     = 0.0
        self.t0         = t0
        self.time_his   = []
        self.curr_time  = rospy.get_rostime().to_sec() - self.t0

    def enc_callback(self,data):
        """Unpack message from sensor, ENC"""
        self.curr_time = rospy.get_rostime().to_sec() - self.t0

        self.v_fl = data.vel_fl
        self.v_fr = data.vel_fr
        self.v_rl = data.vel_bl
        self.v_rr = data.vel_br
        v_est = (self.v_rl + self.v_rr)/2
        if v_est != self.v_prev:
            self.v_meas = v_est
            self.v_prev = v_est
            self.v_count = 0
        else:
            self.v_count += 1
            if self.v_count > 40:
                self.v_meas = 0       

        self.saveHistory()

    def saveHistory(self):
        self.time_his.append(self.curr_time)
        
        self.v_fl_his.append(self.v_fl)
        self.v_fr_his.append(self.v_fr)
        self.v_rl_his.append(self.v_rl)
        self.v_rr_his.append(self.v_rr)

        self.v_meas_his.append(self.v_meas)

class ImuClass(object):
    """ Object collecting GPS measurement data
    Attributes:
        Measurement:
            1.yaw 2.psiDot 3.ax 4.ay 5.roll 6.pitch
        Measurement history:
            1.yaw_his 2.psiDot_his 3.ax_his 4.ay_his 5.roll_his 6.pitch_his
        Time stamp
            1.t0 2.time_his
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """

        rospy.Subscriber('imu/data', Imu, self.imu_callback, queue_size=1)
        input_commands = rospy.Publisher('ecu', ECU, queue_size=1)

        # Imu measurement
        self.running = False
        self.yaw_prev= 0.0
        self.yaw     = 0.0
        self.yawInt  = 0.0
        self.psiDot  = 0.0
        self.ax      = 0.0
        self.ay      = 0.0
        self.az      = 0.0
        self.roll    = 0.0
        self.pitch   = 0.0
        
        # Imu measurement history
        self.yaw_his     = []
        self.psiDot_his  = []
        self.ax_his      = []
        self.ay_his      = []
        self.roll_his    = []
        self.pitch_his   = []
        
        # time stamp
        self.t0          = t0
        self.time_his    = []

        # Time for yawDot integration
        self.curr_time = rospy.get_rostime().to_sec()
        self.prev_time = self.curr_time

    def imu_callback(self,data):
        """Unpack message from sensor, IMU"""
        
        self.curr_time = rospy.get_rostime().to_sec() - self.t0

        if self.prev_time > 0:
            self.yawInt += self.psiDot * (self.curr_time-self.prev_time)
   
        ori = data.orientation
        
        quaternion = (ori.x, ori.y, ori.z, ori.w)
        (roll_raw, pitch_raw, yaw_raw) = transformations.euler_from_quaternion(quaternion)
        self.roll   = roll_raw
        self.pitch  = pitch_raw

        # yaw_meas is element of [-pi,pi]
        yaw = np.unwrap([self.yaw_prev, yaw_raw])[1]       # get smooth yaw (from beginning)
        self.yaw_prev = self.yaw                   # and always use raw measured yaw for unwrapping
        # from this point on 'yaw' will be definitely unwrapped (smooth)!
        if not self.running:
            self.yaw0 = yaw              # set yaw0 to current yaw
            self.yaw = 0                 # and current yaw to zero
            self.running = True
        else:
            self.yaw = yaw - self.yaw0


        w_z = data.angular_velocity.z
        w_x = data.angular_velocity.x
        w_y = data.angular_velocity.y

        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        a_z = data.linear_acceleration.z

        self.psiDot = w_y * sin(roll_raw) / cos(pitch_raw) + w_z * cos(roll_raw) / cos(pitch_raw)
        # Transformation from imu frame to vehicle frame (negative roll/pitch and reversed matrix multiplication to go back)
        # self.ax = cos(-pitch_raw)*a_x + sin(-pitch_raw)*sin(-roll_raw)*a_y - sin(-pitch_raw)*cos(-roll_raw)*a_z
        # self.ay = cos(-roll_raw)*a_y + sin(-roll_raw)*a_z
        self.ax = a_x
        self.ay = a_y
        self.az = a_z

        self.prev_time = self.curr_time


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass