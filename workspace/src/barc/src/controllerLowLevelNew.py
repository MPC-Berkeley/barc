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
            if msg.servo <=  -0.263746610867 :
                self.servo_pwm = (float(msg.servo) +  0.753157914261 ) /  0.0078937306999
            elif msg.servo >=  -0.263746610867  and msg.servo <=  -0.240727897846 :
                self.servo_pwm = (float(msg.servo) +  0.739466679967 ) /  0.00767290434034
            elif msg.servo >=  -0.240727897846  and msg.servo <=  -0.214033176047 :
                self.servo_pwm = (float(msg.servo) +  0.819113536804 ) /  0.00889824059936
            elif msg.servo >=  -0.214033176048  and msg.servo <=  -0.173009267913 :
                self.servo_pwm = (float(msg.servo) +  0.911439614341 ) /  0.0102559770337
            elif msg.servo >=  -0.173009267913  and msg.servo <=  -0.146347195826 :
                self.servo_pwm = (float(msg.servo) +  0.812898998002 ) /  0.00888735736235
            elif msg.servo >=  -0.146347195826  and msg.servo <=  -0.116381689668 :
                self.servo_pwm = (float(msg.servo) +  0.895484849771 ) /  0.00998850205261
            elif msg.servo >=  -0.116381689668  and msg.servo <=  -0.0899240907955 :
                self.servo_pwm = (float(msg.servo) +  0.804279260347 ) /  0.00881919962409
            elif msg.servo >=  -0.0899240907955  and msg.servo <=  -0.0559677156212 :
                self.servo_pwm = (float(msg.servo) +  1.0067462205 ) /  0.0113187917247
            elif msg.servo >=  -0.0559677156213  and msg.servo <=  -0.0292729338541 :
                self.servo_pwm = (float(msg.servo) +  0.803421605101 ) /  0.00889826058905
            elif msg.servo >=  -0.0292729338541  and msg.servo <=  -0.00546569667052 :
                self.servo_pwm = (float(msg.servo) +  0.796395020881 ) /  0.00881749525318
            elif msg.servo >=  -0.00546569667052  and msg.servo <=  -0.000405779408086 :
                self.servo_pwm = (float(msg.servo) +  1.51838095814 ) /  0.0168663908748
            elif msg.servo >=  -0.000405779408088  and msg.servo <=  0.00180188349624 :
                self.servo_pwm = (float(msg.servo) +  0.397785102187 ) /  0.00441532580865
            elif msg.servo >=  0.00180188349625  and msg.servo <=  0.00925861373544 :
                self.servo_pwm = (float(msg.servo) +  1.3478662898 ) /  0.0149134604784
            elif msg.servo >=  0.00925861373543  and msg.servo <=  0.0290577121146 :
                self.servo_pwm = (float(msg.servo) +  0.891600362515 ) /  0.00989954918957
            elif msg.servo >=  0.0290577121146  and msg.servo <=  0.0441437408851 :
                self.servo_pwm = (float(msg.servo) +  0.672442625716 ) /  0.00754301438528
            elif msg.servo >=  0.0441437408851  and msg.servo <=  0.050378221244 :
                self.servo_pwm = (float(msg.servo) +  0.548131893206 ) /  0.00623448035885
            elif msg.servo >=  0.050378221244  and msg.servo <=  0.0669948977951 :
                self.servo_pwm = (float(msg.servo) +  1.54482272766 ) /  0.0166166765511
            elif msg.servo >=  0.0669948977951  and msg.servo <=  0.0896304625649 :
                self.servo_pwm = (float(msg.servo) +  1.03082999354 ) /  0.0113177823849
            elif msg.servo >=  0.0896304625649  and msg.servo <=  0.125460597819 :
                self.servo_pwm = (float(msg.servo) +  1.09276400083 ) /  0.0119433784181
            elif msg.servo >=  0.125460597819  and msg.servo <=  0.154738223475 :
                self.servo_pwm = (float(msg.servo) +  0.869978674471 ) /  0.00975920855187
            elif msg.servo >=  0.154738223475  and msg.servo <=  0.187575837721 :
                self.servo_pwm = (float(msg.servo) +  0.994578275151 ) /  0.0109458714155
            elif msg.servo >=  0.187575837721  and msg.servo <=  0.213318686482 :
                self.servo_pwm = (float(msg.servo) +  0.739166717684 ) /  0.00858094958708
            elif msg.servo >=  0.213318686482  and msg.servo <=  0.24345660213 :
                self.servo_pwm = (float(msg.servo) +  0.901784192482 ) /  0.0100459718826
            elif msg.servo >=  0.24345660213  and msg.servo <=  0.263440689995 :
                self.servo_pwm = (float(msg.servo) +  0.895636406176 ) /  0.00999204393251
            elif msg.servo >=  0.263440689995 :
                self.servo_pwm = (float(msg.servo) +  0.697519611645 ) /  0.00828414053138

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
