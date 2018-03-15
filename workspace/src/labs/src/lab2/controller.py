#!/usr/bin/env python

import rospy
import time
from barc.msg import ECU
from labs.msg import Z_DynBkMdl 
	
# initialize state
x = 0
y = 0
v_x = 0
v_y = 0
	
# ecu command update
def measurements_callback(data):
 	global x, y, psi, v_x, v_y, psi_dot
	x = data.x
	y = data.y
	psi = data.psi 
	v_x = data.v_x 
	v_y = data.v_y 
	psi_dot = data.psi_dot
	 	
# Insert your PID longitudinal controller here: since you are asked to do longitudinal control,  the steering angle d_f can always be set to zero. Therefore, the control output of your controller is essentially longitudinal acceleration acc.
# ==========PID longitudinal controller=========#
class PID():
	def __init__(self, kp=1, ki=1, kd=1):
		self.kp = kp
		self.ki = ki
		self.kd = kd
	
	def acc_calculate(self, speed_reference, speed_current):
			 
	 	acc = TODO

	 	return acc
	
# ==========end of the controller==============#
	
# controller node
def controller():
	# initialize node
	rospy.init_node('controller', anonymous=True)
	
	# topic subscriptions / publications
	rospy.Subscriber('z_vhcl', Z_DynBkMdl, measurements_callback)
	state_pub = rospy.Publisher('ecu', ECU, queue_size = 10)
	
	# set node rate
	loop_rate = 50
	dt = 1.0 / loop_rate
	rate = rospy.Rate(loop_rate)
	t0 = time.time()
	
	# set initial conditions 
	d_f = 0
	acc = 0
	
	# reference speed 
	v_ref = 8 # reference speed is 8 m/s
	
	# Initialize the PID controller with your tuned gains
	PID_control = PID(kp=0.5, ki=0.5, kd=0.5)
	
	while not rospy.is_shutdown():
		# acceleration calculated from PID controller.
	 	acc = PID_control.acc_calculate(v_ref, v_x)
	 
	 	# steering angle
	 	d_f = 0.0
	
		# publish information
	 	state_pub.publish( ECU(acc, d_f) )
	
		# wait
		rate.sleep()
	
if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		pass
