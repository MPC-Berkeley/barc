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
from barc.msg import ECU
import rospy

class low_level_control(object):
    motor_pwm = 90
    servo_pwm = 90
    str_ang_max = 30
    str_ang_min = -40
    ecu_pub = 0
    ecu_cmd = ECU()
    sel_car = rospy.get_param("/low_level_controller/car")

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
        if self.sel_car == "OldBARC":
            if msg.servo <=  -0.289253621435 :
                self.servo_pwm = (float(msg.servo) +  0.632099117661 ) /  0.00601483326712
            elif msg.servo >=  -0.289253621435  and msg.servo <=  -0.248430466181 :
                self.servo_pwm = (float(msg.servo) +  1.06489357127 ) /  0.0136077184182
            elif msg.servo >=  -0.248430466181  and msg.servo <=  -0.224283978217 :
                self.servo_pwm = (float(msg.servo) +  0.731360225456 ) /  0.00804882932126
            elif msg.servo >=  -0.224283978217  and msg.servo <=  -0.186276901771 :
                self.servo_pwm = (float(msg.servo) +  1.02243258359 ) /  0.0126690254821
            elif msg.servo >=  -0.186276901771  and msg.servo <=  -0.162277586312 :
                self.servo_pwm = (float(msg.servo) +  0.714261841851 ) /  0.00799977181941
            elif msg.servo >=  -0.162277586312  and msg.servo <=  -0.125381189409 :
                self.servo_pwm = (float(msg.servo) +  1.01089471508 ) /  0.0122987989677
            elif msg.servo >=  -0.125381189409  and msg.servo <=  -0.0964165243768 :
                self.servo_pwm = (float(msg.servo) +  0.82053315019 ) /  0.00965488834418
            elif msg.servo >=  -0.0964165243768  and msg.servo <=  -0.0581111939358 :
                self.servo_pwm = (float(msg.servo) +  1.0540497854 ) /  0.0127684434803
            elif msg.servo >=  -0.0581111939358  and msg.servo <=  -0.0264038488127 :
                self.servo_pwm = (float(msg.servo) +  0.882502167137 ) /  0.010569115041
            elif msg.servo >=  -0.0264038488127  and msg.servo <=  0.00335828771009 :
                self.servo_pwm = (float(msg.servo) +  0.829981534927 ) /  0.00992071217425
            elif msg.servo >=  0.0033582877101  and msg.servo <=  0.0135350330475 :
                self.servo_pwm = (float(msg.servo) +  0.851488320632 ) /  0.0101767453374
            elif msg.servo >=  0.0135350330475  and msg.servo <=  0.0337526978462 :
                self.servo_pwm = (float(msg.servo) +  0.845715720898 ) /  0.0101088323994
            elif msg.servo >=  0.0337526978462  and msg.servo <=  0.0611985733503 :
                self.servo_pwm = (float(msg.servo) +  0.762177691771 ) /  0.00914862516801
            elif msg.servo >=  0.0611985733503  and msg.servo <=  0.103879535991 :
                self.servo_pwm = (float(msg.servo) +  1.21923030586 ) /  0.0142269875468
            elif msg.servo >=  0.103879535991  and msg.servo <=  0.138433185106 :
                self.servo_pwm = (float(msg.servo) +  0.967283586583 ) /  0.0115178830384
            elif msg.servo >=  0.138433185106  and msg.servo <=  0.171366020895 :
                self.servo_pwm = (float(msg.servo) +  0.915417560161 ) /  0.0109776119299
            elif msg.servo >=  0.171366020895  and msg.servo <=  0.210023753861 :
                self.servo_pwm = (float(msg.servo) +  1.10433916699 ) /  0.0128859109887
            elif msg.servo >=  0.210023753861  and msg.servo <=  0.24686619954 :
                self.servo_pwm = (float(msg.servo) +  1.04261939922 ) /  0.0122808152263
            elif msg.servo >=  0.24686619954  and msg.servo <=  0.279053978444 :
                self.servo_pwm = (float(msg.servo) +  0.879706062075 ) /  0.0107292596344
            elif msg.servo >=  0.279053978444 :
                self.servo_pwm = (float(msg.servo) +  0.850296187645 ) /  0.0104569459823


            # self.servo_pwm = 81.4 + 89.1*float(msg.servo)
        elif self.sel_car == "NewBARC":
            if msg.servo <=  -0.26521905473 :
                self.servo_pwm = (float(msg.servo) +  0.692139987399 ) /  0.00790594319757
            elif msg.servo >=  -0.26521905473  and msg.servo <=  -0.238983411051 :
                self.servo_pwm = (float(msg.servo) +  0.737460640953 ) /  0.00874521455969
            elif msg.servo >=  -0.238983411051  and msg.servo <=  -0.21740608804 :
                self.servo_pwm = (float(msg.servo) +  0.648952548255 ) /  0.00719244100359
            elif msg.servo >=  -0.21740608804  and msg.servo <=  -0.195496295406 :
                self.servo_pwm = (float(msg.servo) +  0.655601940715 ) /  0.00730326421125
            elif msg.servo >=  -0.195496295406  and msg.servo <=  -0.166428504575 :
                self.servo_pwm = (float(msg.servo) +  0.805919902862 ) /  0.00968926361041
            elif msg.servo >=  -0.166428504575  and msg.servo <=  -0.13790460426 :
                self.servo_pwm = (float(msg.servo) +  0.793954311516 ) /  0.00950796677184
            elif msg.servo >=  -0.13790460426  and msg.servo <=  -0.107679261255 :
                self.servo_pwm = (float(msg.servo) +  0.83308749337 ) /  0.0100751143349
            elif msg.servo >=  -0.107679261255  and msg.servo <=  -0.0834305902096 :
                self.servo_pwm = (float(msg.servo) +  0.689647366339 ) /  0.00808289034839
            elif msg.servo >=  -0.0834305902097  and msg.servo <=  -0.0512669256202 :
                self.servo_pwm = (float(msg.servo) +  0.887522204945 ) /  0.0107212215298
            elif msg.servo >=  -0.0512669256202  and msg.servo <=  -0.0176173137472 :
                self.servo_pwm = (float(msg.servo) +  0.926156834318 ) /  0.011216537291
            elif msg.servo >=  -0.0176173137472  and msg.servo <=  0.00548614479964 :
                self.servo_pwm = (float(msg.servo) +  0.641410694513 ) /  0.00770115284896
            elif msg.servo >=  0.00548614479965  and msg.servo <=  0.0395765419165 :
                self.servo_pwm = (float(msg.servo) +  0.949044974472 ) /  0.0113634657056
            elif msg.servo >=  0.0395765419165  and msg.servo <=  0.0526324569052 :
                self.servo_pwm = (float(msg.servo) +  0.339044992756 ) /  0.0043519716629
            elif msg.servo >=  0.0526324569052  and msg.servo <=  0.0528413506089 :
                self.servo_pwm = (float(msg.servo) +  -0.0463656457939 ) /  6.96312345701e-05
            elif msg.servo >=  0.0528413506089  and msg.servo <=  0.079616715143 :
                self.servo_pwm = (float(msg.servo) +  0.777194949949 ) /  0.00892512151137
            elif msg.servo >=  0.079616715143  and msg.servo <=  0.103179066597 :
                self.servo_pwm = (float(msg.servo) +  0.674378531369 ) /  0.00785411715116
            elif msg.servo >=  0.103179066597  and msg.servo <=  0.133318345683 :
                self.servo_pwm = (float(msg.servo) +  0.891417143274 ) /  0.0100464263623
            elif msg.servo >=  0.133318345683  and msg.servo <=  0.16432338677 :
                self.servo_pwm = (float(msg.servo) +  0.920853051245 ) /  0.0103350136954
            elif msg.servo >=  0.16432338677  and msg.servo <=  0.197338456816 :
                self.servo_pwm = (float(msg.servo) +  0.991204064839 ) /  0.0110050233487
            elif msg.servo >=  0.197338456816  and msg.servo <=  0.231381039336 :
                self.servo_pwm = (float(msg.servo) +  0.721811271248 ) /  0.00851064563022
            elif msg.servo >=  0.231381039336  and msg.servo <=  0.250328630703 :
                self.servo_pwm = (float(msg.servo) +  0.475995705006 ) /  0.00631586378878
            elif msg.servo >=  0.250328630703  and msg.servo <=  0.27953185246 :
                self.servo_pwm = (float(msg.servo) +  0.869128203325 ) /  0.00973440725241
            elif msg.servo >=  0.27953185246 :
                self.servo_pwm = (float(msg.servo) +  0.146081068484 ) /  0.00360688916054

            # if msg.servo >= 0:
            #     self.servo_pwm = (float(msg.servo) ) / 0.00822442841151 + 83.3
            # else:
            #     self.servo_pwm = (float(msg.servo) ) / 0.00890579042074 + 83.3
            # Old Map
            # self.servo_pwm = 83.3 + 103.1*float(msg.servo)

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
