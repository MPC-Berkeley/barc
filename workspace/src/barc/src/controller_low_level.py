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
from barc.msg import ECU, pos_info
import rospy

def ESCacc(vel, Acc):

    # Acc_1 = 0.9043 * Acc + 0.09113
    Acc_1 = 0.7043 * Acc + 0.09113

    if vel < 0.1:
        PWM = (26.149 + Acc_1) / 0.2627

    elif (0.1 <= vel < 0.3):
        PWM = (25.396 + Acc_1) / 0.2566

    elif (0.3 <= vel < 0.5):
        PWM = (24.581 + Acc_1) / 0.2453

    elif (0.5 <= vel < 0.7):
        PWM = (24.455 + Acc_1) / 0.243

    elif (0.7 <= vel < 0.9):
        PWM = (24.163 + Acc_1) / 0.2391

    elif (0.9 <= vel < 1.1):
        PWM = (23.75 + Acc_1) / 0.234

    elif (1.1 <= vel < 1.3):
        PWM = (23.048 + Acc_1) / 0.2262

    elif (1.3 <= vel < 1.5):
        PWM = (22.493 + Acc_1) / 0.2198

    elif (1.5 <= vel  < 1.7):
        PWM = (22.368 + Acc_1) / 0.2174

    elif (1.7 <= vel < 1.9):
        PWM = (21.134 + Acc_1) / 0.2045

    elif (1.9 <= vel < 2.1):
        PWM = (19.881 + Acc_1) / 0.1915

    elif (2.1 <= vel < 2.3):
        PWM = (18.258 + Acc_1) / 0.1751

    elif (2.3 <= vel < 2.5):
        PWM = (13.228 + Acc_1) / 0.1265

    else:
        PWM = (10.602 + Acc_1) / 0.1006

    return PWM

###########################################

def ESCdec(vel, Acc):

    Acc_1 = 1.4971 * Acc + 0.1971 

    if vel < 0.1:
        PWM = 88.34 * Acc_1 + 145.41

    elif (0.1 <= vel < 0.3):
        PWM = 70.92 * Acc_1 + 128.22

    elif (0.3 <= vel < 0.5):
        PWM = 58.14 * Acc_1 + 126.61

    elif (0.5 <= vel < 0.7):
        PWM = 37.45 * Acc_1 + 102.98

    elif (0.7 <= vel < 0.9):
        PWM = 23.31 * Acc_1 + 92.23

    elif (0.9 <= vel < 1.1):
        PWM = 6.58 * Acc_1 + 79.61

    elif (1.1 <= vel < 1.3):
        PWM = 29.85 * Acc_1 + 107.69

    elif (1.3 <= vel < 1.5):
        PWM =  30.03 * Acc_1 + 108.73

    elif (1.5 <= vel < 1.7):
        PWM = 37.45 * Acc_1 + 115.3

    else:
        PWM = 22 * Acc_1 + 100

    return PWM

class low_level_control(object):
    motor_pwm = 90
    servo_pwm = 90
    str_ang_max = 35
    str_ang_min = -35
    ecu_pub = 0
    ecu_cmd = ECU()
    vx = 0

    def vx_callback(self, data):
        self.vx = data.v_x

    def pwm_converter_callback(self, msg):
        # translate from SI units in vehicle model
        # to pwm angle units (i.e. to send command signal to actuators)
        # convert desired steering angle to degrees, saturate based on input limits

        # Old servo control:
        # self.servo_pwm = 91.365 + 105.6*float(msg.servo)
        # New servo Control
        # if msg.servo < 0.0:         # right curve
        #     self.servo_pwm = 95.5 + 118.8*float(msg.servo)
        # elif msg.servo > 0.0:       # left curve
        #     # self.servo_pwm = 90.8 + 78.9*float(msg.servo)
        #     self.servo_pwm = 90.0 + 89.0*float(msg.servo)
        self.servo_pwm = 90.0 + 89.0*float(msg.servo)

        # compute motor command
        FxR = float(msg.motor)
        if FxR == 0:
            self.motor_pwm = 90.0
        elif FxR > 0:
            #self.motor_pwm = max(94,91 + 6.5*FxR)   # using writeMicroseconds() in Arduino
            self.motor_pwm = 91 + 6.5*FxR   # using writeMicroseconds() in Arduino
            # self.motor_pwm = ESCacc(self.vx, FxR)
            # self.motor_pwm = max(94,90.74 + 6.17*FxR)
            #self.motor_pwm = 90.74 + 6.17*FxR
            
            #self.motor_pwm = max(94,90.12 + 5.24*FxR)
            #self.motor_pwm = 90.12 + 5.24*FxR
            # Note: Barc doesn't move for u_pwm < 93
        else:               # motor break / slow down
            self.motor_pwm = 93.5 + 46.73*FxR
            # self.motor_pwm = ESCdec(self.vx, FxR)

            # self.motor_pwm = 98.65 + 67.11*FxR
            #self.motor = 69.95 + 68.49*FxR
        self.update_arduino()
    def neutralize(self):
        self.motor_pwm = 40             # slow down first
        self.servo_pwm = 90
        self.update_arduino()
        rospy.sleep(1)                  # slow down for 1 sec
        self.motor_pwm = 90
        self.update_arduino()
    def update_arduino(self):
        self.ecu_cmd.header.stamp = get_rostime()
        self.ecu_cmd.motor = self.motor_pwm
        self.ecu_cmd.servo = self.servo_pwm
        self.ecu_pub.publish(self.ecu_cmd)

def arduino_interface():
    # launch node, subscribe to motorPWM and servoPWM, publish ecu
    init_node('arduino_interface')
    llc = low_level_control()

    Subscriber('ecu', ECU, llc.pwm_converter_callback, queue_size = 1)
    Subscriber('pos_info', pos_info, llc.vx_callback, queue_size=1)

    llc.ecu_pub = Publisher('ecu_pwm', ECU, queue_size = 1)

    # Set motor to neutral on shutdown
    on_shutdown(llc.neutralize)

    # process callbacks and keep alive
    spin()

#############################################################
if __name__ == '__main__':
    try:
        arduino_interface()
    except ROSInterruptException:
        pass
