#!/usr/bin/env python

import rospy
import time
from barc.msg import ECU, Encoder
from numpy import pi
import numpy as np

# from encoder
v_meas      = 0.0
t0          = time.time()
t_start     = time.time()
ang_km1     = 0.0
ang_km2     = 0.0
n_FL        = 0.0
n_FR        = 0.0
n_BL        = 0.0
n_BR        = 0.0
r_tire      = 0.05 # radius of the tire
servo_pwm   = 1600.0
motor_pwm   = 1500.0
motor_pwm_offset = 1500.0
servo_pwm_offset = 1600.0
# reference speed 
v_ref = 3 # reference speed is 3 m/s

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
    n_mean = (n_FL + n_FR + n_BL + n_BR)/4

    # transfer the encoder measurement to angular displacement
    ang_mean = n_mean*2*pi/8

    # compute time elapsed
    tf = time.time()
    dt = tf - t0
    
    # compute speed with second-order, backwards-finite-difference estimate
    v_meas    = r_tire*(ang_mean - 4*ang_km1 + 3*ang_km2)/(2*dt)
    rospy.logwarn("speed = {}".format(v_meas))
    # update old data
    ang_km1 = ang_mean
    ang_km2 = ang_km1
    t0      = time.time()


# Insert your PID longitudinal controller here: since you are asked to do longitudinal control, 
# the steering angle d_f can always be set to zero. Therefore, the control output of your controller 
# is essentially longitudinal acceleration acc.
# ===================================PID longitudinal controller================================#
class PID():
    def __init__(self, kp=1, ki=1, kd=1, integrator=0, derivator=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integrator = integrator
        self.derivator = derivator
        self.integrator_max = 10
        self.integrator_min = -10

    def acc_calculate(self, speed_reference, speed_current):
        self.error = speed_reference - speed_current
        
        # Propotional control
        self.P_effect = self.kp*self.error
        
        # Integral control
        self.integrator = self.integrator + self.error
        ## Anti windup
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

# =====================================end of the controller====================================#

# state estimation node
def controller():
    global motor_pwm, servo_pwm, motor_pwm_offset, servo_pwm_offset
    global v_ref, v_meas, t_start
    # initialize node
    rospy.init_node('CorneringStiffnessTest', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('encoder', Encoder, enc_callback)

    ecu_pub   = rospy.Publisher('ecu_pwm', ECU, queue_size = 10)

    # set node rate
    loop_rate   = 50
    rate        = rospy.Rate(loop_rate)

    # Initialize the PID controller
    PID_control = PID(kp=55.0, ki=20.0, kd=0.0)

    while not rospy.is_shutdown():
        # acceleration calculated from PID controller.
        motor_pwm = PID_control.acc_calculate(v_ref, v_meas) + motor_pwm_offset
        # rospy.logwarn("pwm = {}".format(motor_pwm))
        t_now = time.time()
        servo_pwm = 200*np.sin(pi*(t_now - t_start)) + servo_pwm_offset
        rospy.logwarn("pwm = {}".format(servo_pwm))
        # publish information
        ecu_pub.publish( ECU(motor_pwm, servo_pwm) )

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       controller()
    except rospy.ROSInterruptException:
        pass
