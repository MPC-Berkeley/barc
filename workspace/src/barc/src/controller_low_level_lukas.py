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
from state_estimation_SensorKinematicModel_lukas import EncClass, ImuClass
import rospy
import numpy as np
import os

class low_level_control(object):
    motor_pwm = 89
    servo_pwm = 90
    str_ang_max = 35
    str_ang_min = -35
    ecu_pub = 0
    ecu_cmd = ECU()
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
        #     self.servo_pwm = 90.8 + 78.9*float(msg.servo)
        self.servo_pwm = 90.0 + 89.0*float(msg.servo)

        # compute motor command
        FxR = float(msg.motor)
        if FxR == 0:
            self.motor_pwm = 90.0
        elif FxR > 0:
            #self.motor_pwm = max(94,91 + 6.5*FxR)   # using writeMicroseconds() in Arduino
            self.motor_pwm = 91 + 6.5*FxR   # using writeMicroseconds() in Arduino

            # self.motor_pwm = max(94,90.74 + 6.17*FxR)
            #self.motor_pwm = 90.74 + 6.17*FxR
            
            #self.motor_pwm = max(94,90.12 + 5.24*FxR)
            #self.motor_pwm = 90.12 + 5.24*FxR
            # Note: Barc doesn't move for u_pwm < 93
        else:               # motor break / slow down
            self.motor_pwm = 93.5 + 46.73*FxR
            # self.motor_pwm = 98.65 + 67.11*FxR
            #self.motor = 69.95 + 68.49*FxR
        self.update_arduino()
    def neutralize(self):
        self.motor_pwm = 40             # slow down first
        self.servo_pwm = 90
        self.update_arduino()
        rospy.sleep(1)                  # slow down for 1 sec
        self.motor_pwm = 80
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
    llc.ecu_pub = Publisher('ecu_pwm', ECU, queue_size = 1)

    # Set motor to neutral on shutdown
    on_shutdown(llc.neutralize)

    # process callbacks and keep alive
    spin()

def steering_mapping():
    # launch node, subscribe to motorPWM and servoPWM, publish ecu
    init_node('arduino_interface')

    node_name = rospy.get_name()
    mode = rospy.get_param(node_name + "/mode") 
    steering_pwm = rospy.get_param(node_name + "/steering_pwm") 
    acceleration_pwm = rospy.get_param(node_name + "/acceleration_pwm") 
    acceleration_neg_pwm = rospy.get_param(node_name + "/acceleration_neg_pwm") 

    llc = low_level_control()

    llc.ecu_pub = Publisher('ecu_pwm', ECU, queue_size=1)
    state_pub_pos  = rospy.Publisher('pos_info', pos_info, queue_size=1)
    sensor_readings = pos_info()

    # Set motor to neutral on shutdown
    on_shutdown(llc.neutralize)

    t0 = rospy.get_rostime().to_sec()
    imu = ImuClass(t0)
    enc = EncClass(t0)

    loop_rate = 20
    rate = rospy.Rate(loop_rate)
    acceleration_time = 3  # accelerate for four seconds
    max_counts = loop_rate * acceleration_time

    """
    if mode == "steering":
        max_counts = 100

        for j in range(60, 130, 10):
            i = 0
            while not rospy.is_shutdown() and i <= max_counts:
                llc.motor_pwm = acceleration_pwm
                llc.servo_pwm = j
                llc.update_arduino()

                sensor_readings.v_x = enc.v_meas
                sensor_readings.psiDot = imu.psiDot
                state_pub_pos.publish(sensor_readings)

                enc.saveHistory()
                imu.saveHistory()

                i += 1
                rate.sleep()  
    """

    if mode == "steering":
        max_counts = 1500
        i = 0

        while not rospy.is_shutdown() and i <= max_counts:
            llc.motor_pwm = acceleration_pwm
            llc.servo_pwm = steering_pwm
            llc.update_arduino()

            sensor_readings.v_x = enc.v_meas
            sensor_readings.psiDot = imu.psiDot
            state_pub_pos.publish(sensor_readings)

            enc.saveHistory()
            imu.saveHistory()

            i += 1
            rate.sleep()  

    elif mode == "acceleration": 
        
        i = 0

        while not rospy.is_shutdown() and i <= max_counts:
            llc.motor_pwm = acceleration_pwm
            llc.servo_pwm = steering_pwm
            llc.update_arduino()

            sensor_readings.v_x = enc.v_rr
            sensor_readings.psiDot = imu.psiDot
            state_pub_pos.publish(sensor_readings)

            enc.saveHistory()
            imu.saveHistory()

            if enc.v_rr > 1e-5:
                i += 1
            rate.sleep() 

        while not rospy.is_shutdown():
            llc.motor_pwm = acceleration_neg_pwm
            llc.servo_pwm = steering_pwm
            llc.update_arduino()

            sensor_readings.v_x = enc.v_meas
            sensor_readings.psiDot = imu.psiDot
            state_pub_pos.publish(sensor_readings)

            enc.saveHistory()
            imu.saveHistory()

            rate.sleep()  

    homedir = os.path.expanduser("~")
    directory = homedir + "/barc_debugging/" + mode + "/"
    if mode == "acceleration":
        directory_string = str(steering_pwm) + "_" + str(acceleration_pwm) + "_" + str(acceleration_neg_pwm)
    else:
        directory_string = str(steering_pwm) + "_" + str(acceleration_pwm)

    directory = directory + directory_string
    try:
        os.mkdir(directory)
    except: 
        print "Directory already exists"

    pathSave = directory + "/estimator_imu.npz"
    np.savez(pathSave,psiDot_his    = imu.psiDot_his,
                      roll_his      = imu.roll_his,
                      pitch_his     = imu.pitch_his,
                      yaw_his       = imu.yaw_his,
                      ax_his        = imu.ax_his,
                      ay_his        = imu.ay_his,
                      imu_time      = imu.time_his)

    pathSave = directory + "/estimator_enc.npz"
    np.savez(pathSave,v_fl_his          = enc.v_fl_his,
                      v_fr_his          = enc.v_fr_his,
                      v_rl_his          = enc.v_rl_his,
                      v_rr_his          = enc.v_rr_his,
                      v_meas_his        = enc.v_meas_his,
                      enc_time          = enc.time_his)


#############################################################
if __name__ == '__main__':
    try:
        steering_mapping()
        # arduino_interface()
    except ROSInterruptException:
        pass
