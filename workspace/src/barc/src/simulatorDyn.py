#!/usr/bin/env python
"""
    File name: barc_simulator_dyn.py
    Author: Shuqi Xu
    Email: shuqixu@kth.se
    Python Version: 2.7.12
"""
# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
# based on an open source project by Bruce Wootton
#----------------------------------------------------------------------------

import rospy
import geometry_msgs.msg
from barc.msg import ECU, pos_info, Vel_est
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_imu_fusion, hedge_pos
from numpy import tan, arctan, cos, sin, pi
from numpy.random import randn
from tf import transformations

def main():
    rospy.init_node("simulator")
    sim = Simulator()
    imu = ImuClass()
    gps = GpsClass()
    enc = EncClass()
    ecu = EcuClass()

    counter = 0
    counter_gps = 0
    a_his 	= [0.0]*int(rospy.get_param("simulator/delay_a")/rospy.get_param("simulator/dt"))
    df_his 	= [0.0]*int(rospy.get_param("simulator/delay_df")/rospy.get_param("simulator/dt"))
    while not rospy.is_shutdown():
    	# Simulator delay
		a_his.append(ecu.u[0])
		df_his.append(ecu.u[1])
		u = [a_his.pop(0), df_his.pop(0)]

		sim.f(u)

		imu.update(sim)
		gps.update(sim)
		enc.update(sim)

		sim.saveHistory()

		# gps.gps_pub()
		imu.imu_pub()
		sim.sim_pub()

		counter += 1
		counter_gps += 1
		if counter == 5:
			enc.enc_pub()
			counter = 0
		if counter_gps == 6:
			gps.gps_pub()
			counter_gps = 0
		sim.rate.sleep()

class Simulator(object):
	""" Object collecting GPS measurement data
    Attributes:
    	Model params:
    		1.L_f 2.L_r 3.m(car mass) 3.I_z(car inertial) 4.c_f(equivalent drag coefficient)
        States:
            1.x 2.y 3.vx 4.vy 5.ax 6.ay 7.psiDot
        States history:
            1.x_his 2.y_his 3.vx_his 4.vy_his 5.ax_his 6.ay_his 7.psiDot_his
        Simulator sampling time:
        	1. dt
        Time stamp:
        	1. time_his
	Methods:
		f(u):
			System model used to update the states
		pacejka(ang):
			Pacejka lateral tire modeling
    """
	def __init__(self):
		self.pub  = rospy.Publisher("real_val", pos_info, queue_size=1)
		self.L_f = rospy.get_param("L_a")
		self.L_r = rospy.get_param("L_b") 
		self.m 	= rospy.get_param("m")
		self.I_z = rospy.get_param("I_z")
		self.c_f = rospy.get_param("simulator/c_f")

		self.B = rospy.get_param("simulator/B")
		self.C = rospy.get_param("simulator/C")
		self.mu= rospy.get_param("simulator/mu")
		self.g = rospy.get_param("simulator/g")

		self.x 		= 0.02
		self.y 		= 0.0
		self.vx 	= 0.0
		self.vy 	= 0.0
		self.ax 	= 0.0
		self.ay 	= 0.0
		self.msg 	= pos_info()
		
		if rospy.get_param("feature_flag"):
			self.yaw = pi/4
		else:
			self.yaw = 0.0

		self.psiDot = 0.0

		self.x_his 		= []
		self.y_his 		= []
		self.vx_his 	= []
		self.vy_his 	= []
		self.ax_his 	= []
		self.ay_his 	= []
		self.psiDot_his = []

		self.dt 		= rospy.get_param("simulator/dt")
		self.rate 		= rospy.Rate(1.0/self.dt)
		self.time_his 	= []

	def f(self,u):
		a_F = 0.0
		a_R = 0.0

		if abs(self.vx) > 0.2:
			a_F = arctan((self.vy + self.L_f*self.psiDot)/abs(self.vx)) - u[1]
			a_R = arctan((self.vy - self.L_r*self.psiDot)/abs(self.vx))

		FyF = -self.pacejka(a_F)
		FyR = -self.pacejka(a_R)

		if abs(a_F) > 30.0/180.0*pi or abs(a_R) > 30.0/180.0*pi:
			print "WARNING: Large slip angles in simulation"

		x 	= self.x
		y 	= self.y
		ax 	= self.ax
		ay 	= self.ay
		vx 	= self.vx
		vy 	= self.vy
		yaw = self.yaw
		psiDot = self.psiDot

		self.x 		+= self.dt*(cos(yaw)*vx - sin(yaw)*vy)
		self.y 		+= self.dt*(sin(yaw)*vx + cos(yaw)*vy)
		self.vx 	+= self.dt*(ax + psiDot*vy)
		self.vy 	+= self.dt*(ay - psiDot*vx)
		self.ax 	 = u[0] - self.c_f*vx - FyF/self.m*sin(u[1])
		self.ay 	 = 1.0/self.m*(FyF*cos(u[1])+FyR)
		self.yaw 	+= self.dt*(psiDot)                                        
		self.psiDot += self.dt*(1.0/self.I_z*(self.L_f*FyF*cos(u[1]) - self.L_r*FyR))

		self.vx = abs(self.vx)
	
	def pacejka(self,ang):
		D = self.mu*self.m*self.g/2
		C_alpha_f = D*sin(self.C*arctan(self.B*ang))
		return C_alpha_f

	def saveHistory(self):
		self.x_his.append(self.x)
		self.y_his.append(self.y)
		self.vx_his.append(self.vx)
		self.vy_his.append(self.vy)
		self.ax_his.append(self.ax)
		self.ay_his.append(self.ay)
		self.psiDot_his.append(self.psiDot)
		self.time_his.append(rospy.get_rostime().to_sec()) 

	def sim_pub(self):
		self.msg.a_x 	= self.ax
		self.msg.a_y 	= self.ay
		self.msg.psi 	= self.yaw
		self.msg.psiDot = self.psiDot
		self.msg.v_x 	= self.vx
		self.msg.v_y 	= self.vy
		self.pub.publish(self.msg)

class ImuClass(object):
	def __init__(self):
		self.pub  = rospy.Publisher("imu/data", Imu, queue_size=1)
		self.ax 	= 0.0
		self.ay 	= 0.0
		self.yaw 	= 0.0
		self.psiDot = 0.0
		self.ax_std 	= rospy.get_param("simulator/ax_std")
		self.ay_std 	= rospy.get_param("simulator/ay_std")
		self.psiDot_std = rospy.get_param("simulator/psiDot_std")
		self.n_bound 	= rospy.get_param("simulator/n_bound")

		self.msg = Imu()

	def update(self,sim):
		n = self.ax_std*randn()
		n = min(n, self.ax_std*self.n_bound)
		n = max(n,-self.ax_std*self.n_bound)
		self.ax = sim.ax + n

		n = self.ay_std*randn()
		n = min(n, self.ay_std*self.n_bound)
		n = max(n,-self.ay_std*self.n_bound)
		self.ay = sim.ay + n

		n = self.psiDot_std*randn()
		n = min(n, self.psiDot_std*self.n_bound)
		n = max(n,-self.psiDot_std*self.n_bound)
		self.psiDot = sim.psiDot + n

		self.yaw = sim.yaw

	def imu_pub(self):
		self.msg.linear_acceleration.x = self.ax
		self.msg.linear_acceleration.y = self.ay
		self.msg.angular_velocity = Vector3(0,0,self.psiDot)
		self.orientation = geometry_msgs.msg.Quaternion(0, 0, sin(self.yaw/2), cos(self.yaw/2))
		self.pub.publish(self.msg)

class GpsClass(object):
	def __init__(self):
		self.pub  = rospy.Publisher("hedge_pos", hedge_pos, queue_size=1)
		self.x = 0.0
		self.y = 0.0
		self.x_std 	 = rospy.get_param("simulator/x_std")
		self.y_std 	 = rospy.get_param("simulator/y_std")
		self.n_bound = rospy.get_param("simulator/n_bound")

		self.msg = hedge_pos()

	def update(self,sim):
		n = self.x_std*randn()
		n = min(n, self.x_std*self.n_bound)
		n = max(n,-self.x_std*self.n_bound)
		self.x = sim.x + n

		n = self.y_std*randn()
		n = min(n, self.y_std*self.n_bound)
		n = max(n,-self.y_std*self.n_bound)
		self.y = sim.y + n

	def gps_pub(self):
		self.msg.x_m = self.x
		self.msg.y_m = self.y
		self.pub.publish(self.msg)

class EncClass(object):
	def __init__(self):
		self.pub  = rospy.Publisher("vel_est", Vel_est, queue_size=1)
		self.v = 0.0
		self.v_std 	 = rospy.get_param("simulator/v_std")
		self.n_bound = rospy.get_param("simulator/n_bound")

		self.msg = Vel_est()

	def update(self,sim):
		n = self.v_std*randn()
		n = min(n, self.v_std*self.n_bound)
		n = max(n,-self.v_std*self.n_bound)
		self.v = (sim.vx**2+sim.vy**2)**0.5 + n

	def enc_pub(self):
		self.msg.vel_fl  = self.v
		self.msg.vel_fr  = self.v
		self.msg.vel_bl  = self.v
		self.msg.vel_br  = self.v
		self.msg.vel_est = self.v
		self.pub.publish(self.msg)

class EcuClass(object):
	def __init__(self):
		self.sub = rospy.Subscriber("ecu", ECU, self.ecu_callback, queue_size=1)
		self.u = [0.0, 0.0]

	def ecu_callback(self,data):
		self.u = [data.motor, data.servo]

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass