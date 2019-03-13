#!/usr/bin/env python

import rospy
import time
from barc.msg import ECU, Encoder
from numpy import pi

# from encoder
v_meas      = 0.0
t0          = time.time()
r_tire      = 0.05 # radius of the tire
servo_pwm   = 1580.0
motor_pwm   = 1500.0
motor_pwm_offset = 1500.0

# reference speed 
v_ref = 0.5 # reference speed is 0.5 m/s

# encoder measurement update
def enc_callback(data):
    global t0, v_meas
    global n_FL, n_FR, n_BL, n_BR
    global ang_km1, ang_km2

    n_FL = data.FL
    n_FR = data.FR
    n_BL = data.BL
    n_BR = data.BR

    # compute the average encoder measurement
    n_mean = (n_FL + n_FR)/2

    # transfer the encoder measurement to angular displacement
    ang_mean = n_mean*2*pi/8

    # compute time elapsed
    tf = time.time()
    dt = tf - t0
    
    # compute speed with second-order, backwards-finite-difference estimate
    v_meas    = r_tire*(ang_mean - 4*ang_km1 + 3*ang_km2)/(dt)
    rospy.logwarn("velocity = {}".format(v_meas))
    # update old data
    ang_km1 = ang_mean
    ang_km2 = ang_km1
    t0      = time.time()




# ===================================PID longitudinal controller================================#
class PID():
    def __init__(self, kp=1, ki=1, kd=1, integrator=0, derivator=0):
        self.kp = kp
        self.ki = ki
        self.integrator = integrator
        self.integrator_max = 100

    def acc_calculate(self, speed_reference, speed_current):
        self.error = speed_reference - speed_current
        
        # Propotional control
        self.P_effect = self.kp*self.error
        
        # Integral control
        self.integrator = self.integrator + self.error
        
		# Anti windup
        if self.integrator >= self.integrator_max:
           self.integrator = self.integrator_max

        self.I_effect = self.ki*self.integrator

        acc = self.P_effect + self.I_effect

        return acc

# =====================================end of the controller====================================#

# state estimation node
def controller():
    global motor_pwm, servo_pwm, motor_pwm_offset
    global v_ref, v_meas
    
    # Initialize node:
    rospy.init_node('simulationGain', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('encoder', Encoder, enc_callback)
    # TODO: Add your necessary topic subscriptions / publications, depending on your preferred method of velocity estimation
    ecu_pub   = rospy.Publisher('ecu_pwm', ECU, queue_size = 10)

    # Set node rate
    loop_rate   = 50
    rate        = rospy.Rate(loop_rate)
    # Initialize the PID controller
    PID_control = PID(kp = 1, ki = 1, kd = 0)
    
    while not rospy.is_shutdown():
        # calculate acceleration from PID controller.
        motor_pwm = PID_control.acc_calculate(v_ref, v_meas) + motor_pwm_offset
        ecu_pub.publish( ECU(motor_pwm, servo_pwm) )

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       controller()
    except rospy.ROSInterruptException:
        pass
