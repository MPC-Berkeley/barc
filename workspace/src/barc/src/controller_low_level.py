#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed at UC
# Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu) and Greg Marcil (grmarcil@berkeley.edu). The cloud
# services integation with ROS was developed by Kiet Lam
# (kiet.lam@berkeley.edu). The web-server app Dator was based on an open source
# project by Bruce Wootton
# ---------------------------------------------------------------------------

# README: This node serves as an outgoing messaging bus from odroid to arduino
# Subscribes: steering and motor commands on 'ecu'
# Publishes: combined ecu commands as 'ecu_pwm'

from rospy import init_node, Subscriber, Publisher, get_param, get_rostime
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from barc.msg import ECU, Vel
import rospy
from pid import PID
import matplotlib.pyplot as plt

class low_level_control(object):
    motor_pwm = 90
    servo_pwm = 90
    str_ang_max = 35
    str_ang_min = -35
    ecu_pub = 0
    ecu_cmd = ECU()
    dt = 0.1
    cm = 0.5
    p = 0.0
    i = 0.0
    d = 0.0
    u_ff = 90.0
    v_enc = 0.0

    error_vals = []
    time_vals = []
    ti = 0.0

    # Set the coefficients for the motor model. 
    def __init__(self, motor_c0, motor_c1, servo_c0, servo_c1):
        self.p = 7.0
        self.i = 0.0
        self.d = 0.0
        # self.pid = PID(P=self.p, I=self.i, D=self.d)

        # plt.figure()
        # plt.ion()
        # self.motor_c0 = motor_c0
        # self.motor_c1 = motor_c1
        # self.servo_c0 = servo_c0
        # self.servo_c1 = servo_c1
        # self.v_enc = 0.0


    def encoder_vel_callback(self, msg):
        self.v_enc = msg.v

    def pwm_converter_callback(self, msg):

        d_f = float(msg.servo)
        acc = float(msg.motor)
        
        # mapping from steering angle to servo input        
        # self.servo_pwm = self.servo_c0 + self.servo_c1 * d_f
        
        self.servo_pwm = 25.3769*(d_f)**2 - 85.3010*d_f + 87.2525
        
        # self.servo_pwm = 11.2*(d_f)**2 - 92.4*d_f + 92.1
        #self.servo_pwm = -92.4*(d_f) + 93.6

        
        # #PID controller 
        # v_ref = acc - self.cm * self.v_enc
        # print "v_ref: ", v_ref
        # self.pid.setPoint(v_ref)
        # e = self.dt * v_ref
        
        # u_fbk = self.pid.update(e, self.dt)

        # print "PWM: ", u_fbk + 90

        # self.motor_pwm = self.u_ff + u_fbk


        # self.error_vals.append(e)
        # self.time_vals.append(self.ti)

        # self.ti += self.dt

        # print(self.error_vals)
        # plt.plot(self.time_vals, self.error_vals, 'r-')
        # plt.title('Error signal')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Error')
        # plt.show()
        # # plt.pause(0.001)
        # # plt.gcf().clear()

        # # mapping from acceleration to motor input
        
        # if acc == 0:
        #     self.motor_pwm = 90.0
        # else:
        #     #self.motor_pwm = 90.0 + self.motor_c0 + self.motor_c1 * acc
        #     self.motor_pwm = ((acc - self.motor_c0) / self.motor_c1) + 90
        # print "acc: ", acc
        # if acc == 0:
        #     self.motor_pwm = 90.0
        # elif acc > 0:
        #     self.motor_pwm = 90 + 13.55 * acc
        # else:
        #     self.motor_pwm = 93.5 + 46.73*acc
 

        if (acc >= -2 and acc <= 0):
            self.motor_pwm = 34.69 * acc + 80.61
        elif (acc > 0 and acc <= 4):
            self.motor_pwm = 1.779 * acc + 94.97
        else:
            self.motor_pwm = 90

        # if acc == 0:
        #     self.motor_pwm = 90.0
        # elif acc > 0:
        #     self.motor_pwm = 91 + 6.5 * acc
        # else:
        #     self.motor_pwm = 93.5 + 46.73*acc

        self.update_arduino()
    

    def update_arduino(self):
        self.ecu_cmd.header.stamp = get_rostime()
        self.ecu_cmd.motor = self.motor_pwm
        self.ecu_cmd.servo = self.servo_pwm
        self.ecu_pub.publish(self.ecu_cmd)

def arduino_interface():
    # launch node, subscribe to motorPWM and servoPWM, publish ecu
    init_node('arduino_interface')

    motor_c0 = get_param("low_level_controller/motor_c0")
    motor_c1 = get_param("low_level_controller/motor_c1")
    servo_c0 = get_param("low_level_controller/servo_c0")
    servo_c1 = get_param("low_level_controller/servo_c1")
    llc = low_level_control(motor_c0, motor_c1, servo_c0, servo_c1)

    Subscriber('ecu', ECU, llc.pwm_converter_callback, queue_size = 1)
    #Subscriber('encoder_vel', Vel, llc.encoder_vel_callback, queue_size=1)
    llc.ecu_pub = Publisher('ecu_pwm', ECU, queue_size = 1)
    # plt.figure()
    # plt.ion()
    # plt.plot(self.time_vals, self.error_vals, 'r-')
    # plt.title('Error signal')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Error')
    # plt.show()
    # plt.pause(0.001)
    # plt.gcf().clear()

    # process callbacks and keep alive
    spin()

#############################################################
if __name__ == '__main__':
    try:
        arduino_interface()
    except ROSInterruptException:
        pass
