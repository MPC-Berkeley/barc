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
import numpy as np

class low_level_control(object):
    motor_pwm = 90
    servo_pwm = 90
    str_ang_max = 30
    str_ang_min = -40
    ecu_pub = 0
    ecu_cmd = ECU()
    sel_car = rospy.get_param("/low_level_controller/car")
    recorded_servo = [0.0]*int(10)
    fbk_servo = 0.0
    commanded_steering = 0.0

    def fbk_servo_callback(self,data ):
        """Unpack message from sensor, ENC"""

        self.recorded_servo.pop(0)
        self.recorded_servo.append(data.servo)

        self.value_servo = np.sum(self.recorded_servo)/len(self.recorded_servo)

        self.fbk_servo = -0.0031283647115524587 * self.value_servo +  1.0745666280004817

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
        self.commanded_steering = msg.servo

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
            if msg.servo >=  0.272459514839 :
                self.servo_pwm = (float(msg.servo) +  -0.574154083239 ) /  -0.00529288716491
            elif msg.servo <=  0.272459514839  and msg.servo >=  0.252464054654 :
                self.servo_pwm = (float(msg.servo) +  -0.652373258353 ) /  -0.00666515339499
            elif msg.servo <=  0.252464054654  and msg.servo >=  0.231712812078 :
                self.servo_pwm = (float(msg.servo) +  -0.667488906177 ) /  -0.00691708085871
            elif msg.servo <=  0.231712812078  and msg.servo >=  0.211911798974 :
                self.servo_pwm = (float(msg.servo) +  -0.647534087242 ) /  -0.00660033770102
            elif msg.servo <=  0.211911798974  and msg.servo >=  0.191666483452 :
                self.servo_pwm = (float(msg.servo) +  -0.657308740465 ) /  -0.00674843850744
            elif msg.servo <=  0.191666483452  and msg.servo >=  0.170119123149 :
                self.servo_pwm = (float(msg.servo) +  -0.687255770435 ) /  -0.00718245343453
            elif msg.servo <=  0.170119123149  and msg.servo >=  0.149027841992 :
                self.servo_pwm = (float(msg.servo) +  -0.676309870917 ) /  -0.00703042705234
            elif msg.servo <=  0.149027841992  and msg.servo >=  0.121506819027 :
                self.servo_pwm = (float(msg.servo) +  -0.66504702257 ) /  -0.00688025574104
            elif msg.servo <=  0.121506819027  and msg.servo >=  0.100073980149 :
                self.servo_pwm = (float(msg.servo) +  -0.685904909481 ) /  -0.007144279626
            elif msg.servo <=  0.100073980149  and msg.servo >=  0.0776850230266 :
                self.servo_pwm = (float(msg.servo) +  -0.712038808172 ) /  -0.00746298570759
            elif msg.servo <=  0.0776850230266  and msg.servo >=  0.0515800538663 :
                self.servo_pwm = (float(msg.servo) +  -0.8173258159 ) /  -0.00870165638675
            elif msg.servo <=  0.0515800538663  and msg.servo >=  0.0360638973609 :
                self.servo_pwm = (float(msg.servo) +  -0.506720644691 ) /  -0.00517205216846
            elif msg.servo <=  0.0360638973609  and msg.servo >=  0.0149161731103 :
                self.servo_pwm = (float(msg.servo) +  -0.677544866298 ) /  -0.00704924141689
            elif msg.servo <=  0.0149161731103  and msg.servo >=  0.00586622560447 :
                self.servo_pwm = (float(msg.servo) +  -0.865611238655 ) /  -0.0090499475058
            elif msg.servo <=  0.00586622560447  and msg.servo >=  -0.0025395705416 :
                self.servo_pwm = (float(msg.servo) +  -0.405141542543 ) /  -0.00420289807304
            elif msg.servo <=  -0.0025395705416  and msg.servo >=  -0.00862030399488 :
                self.servo_pwm = (float(msg.servo) +  -0.587291574427 ) /  -0.00608073345328
            elif msg.servo <=  -0.00862030399488  and msg.servo >=  -0.0233266496672 :
                self.servo_pwm = (float(msg.servo) +  -0.711990633949 ) /  -0.00735317283616
            elif msg.servo <=  -0.0233266496672  and msg.servo >=  -0.0464964529075 :
                self.servo_pwm = (float(msg.servo) +  -0.74900012501 ) /  -0.00772326774677
            elif msg.servo <=  -0.0464964529075  and msg.servo >=  -0.0629780940721 :
                self.servo_pwm = (float(msg.servo) +  -0.519373227078 ) /  -0.00549388038821
            elif msg.servo <=  -0.0629780940721  and msg.servo >=  -0.0892145096345 :
                self.servo_pwm = (float(msg.servo) +  -0.864041922465 ) /  -0.00874547185413
            elif msg.servo <=  -0.0892145096345  and msg.servo >=  -0.107697730485 :
                self.servo_pwm = (float(msg.servo) +  -0.582342514607 ) /  -0.00616107361689
            elif msg.servo <=  -0.107697730485  and msg.servo >=  -0.129557353808 :
                self.servo_pwm = (float(msg.servo) +  -0.70839487357 ) /  -0.00728654110763
            elif msg.servo <=  -0.129557353808  and msg.servo >=  -0.145087166327 :
                self.servo_pwm = (float(msg.servo) +  -0.465752126078 ) /  -0.00517660417292
            elif msg.servo <=  -0.145087166327  and msg.servo >=  -0.171194849622 :
                self.servo_pwm = (float(msg.servo) +  -0.625089490887 ) /  -0.00652692082385
            elif msg.servo <=  -0.171194849622  and msg.servo >=  -0.191099105551 :
                self.servo_pwm = (float(msg.servo) +  -0.638244891461 ) /  -0.00663475197609
            elif msg.servo <=  -0.191099105551  and msg.servo >=  -0.209463703953 :
                self.servo_pwm = (float(msg.servo) +  -0.574092494542 ) /  -0.00612153280074
            elif msg.servo <=  -0.209463703953  and msg.servo >=  -0.230575448718 :
                self.servo_pwm = (float(msg.servo) +  -0.691304072718 ) /  -0.00703724825524
            elif msg.servo <=  -0.230575448718  and msg.servo >=  -0.246365227982 :
                self.servo_pwm = (float(msg.servo) +  -0.458911579136 ) /  -0.00526325975461
            elif msg.servo <=  -0.246365227982  and msg.servo >=  -0.267436556835 :
                self.servo_pwm = (float(msg.servo) +  -0.694820794115 ) /  -0.00702377628431
            elif msg.servo <=  -0.267436556835 :
                self.servo_pwm = (float(msg.servo) +  -0.494743806388 ) /  -0.0055633603155


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
    Subscriber('srv_fbk', ECU, llc.fbk_servo_callback, queue_size = 1)
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
