#!/usr/bin/env python

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
# ---------------------------------------------------------------------------

import rospy
import time
from barc.msg import ECU
from simulator.msg import Z_DynBkMdl, eZ_DynBkMdl 

# from encoder
x        = 0
y        = 0
s        = 0
ey       = 0 
epsi     = 0                     # counts in the front left tire
psi_dot  = 0                     # counts in the front right tire
v_x      = 0
v_y      = 0

# ecu command update
def measurements_callback(data):
    global x, y, psi, v_x, v_y, psi_dot
    x        = data.x          # input acceleration
    y        = data.y          # input steering angle
    psi      = data.psi        # input acceleration
    v_x      = data.v_x        # input acceleration
    v_y      = data.v_y        # input steering angle
    psi_dot  = data.psi_dot    # input steering angle
    
def measurements_error_frame_callback(data):
    global s, ey, epsi, v_x, v_y, psi_dot
    s        = data.s          # input acceleration
    ey       = data.ey          # input steering angle
    epsi     = data.epsi        # input acceleration
    v_x      = data.v_x        # input acceleration
    v_y      = data.v_y        # input steering angle
    psi_dot  = data.psi_dot    # input steering angle

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

    # initialize node
    rospy.init_node('vehicle_simulator', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('z_vhcl', Z_DynBkMdl, measurements_callback)
    rospy.Subscriber('ez_vhcl', eZ_DynBkMdl, measurements_error_frame_callback)

    state_pub   = rospy.Publisher('ecu', ECU, queue_size = 10)

    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    # set initial conditions 
    d_f = 0
    acc = 0

    # reference speed 
    v_ref = 8 # reference speed is 8 m/s

    # Initialize the PID controller
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
