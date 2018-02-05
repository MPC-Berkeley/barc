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
		self.integrator = 0
		self.derivator = 0
		self.integrator_max = 10
		self.integrator_min = -10
	
	def acc_calculate(self, speed_reference, speed_current):
		
		self.error = speed_reference - speed_current
	 
	 	# Propotional control
	 	self.P_effect = self.kp*self.error
	 
		# Integral control with anti-windup
		self.integrator = self.integrator + self.error
	 	if self.integrator >= self.integrator_max:
	 		self.integrator = self.integrator_max
	 	if self.integrator <= self.integrator_min:
	 		self.integrator = self.integrator_min
	 	self.I_effect = self.ki*self.integrator
	 
	 	# Derivative control
	 	self.derivator = self.error - self.derivator
	 	self.D_effect = self.kd*self.derivator
	 	self.derivator = self.error
	
	 	acc = self.P_effect + self.I_effect + self.D_effect
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
	
	PID_control = PID(kp=10, ki=0.5, kd=0.4)
	
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
