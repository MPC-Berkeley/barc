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
            if msg.servo <=  0.27245951483865083 :
                self.servo_pwm = (float(msg.servo) +  -0.5741540832387622 ) /  -0.005292887164914234
            elif msg.servo >=  0.2724595148386455  and msg.servo <=  0.2524640546536816 :
                self.servo_pwm = (float(msg.servo) +  -0.6523732583529593 ) /  -0.0066651533949879625
            elif msg.servo >=  0.25246405465368715  and msg.servo <=  0.23171281207754646 :
                self.servo_pwm = (float(msg.servo) +  -0.6674889061765009 ) /  -0.006917080858713563
            elif msg.servo >=  0.23171281207754973  and msg.servo <=  0.21191179897448864 :
                self.servo_pwm = (float(msg.servo) +  -0.6475340872418329 ) /  -0.006600337701020368
            elif msg.servo >=  0.21191179897448997  and msg.servo <=  0.19166648345218062 :
                self.servo_pwm = (float(msg.servo) +  -0.6573087404652957 ) /  -0.006748438507436451
            elif msg.servo >=  0.19166648345217757  and msg.servo <=  0.17011912314857736 :
                self.servo_pwm = (float(msg.servo) +  -0.6872557704349829 ) /  -0.00718245343453341
            elif msg.servo >=  0.1701191231485717  and msg.servo <=  0.1490278419915394 :
                self.servo_pwm = (float(msg.servo) +  -0.6763098709173473 ) /  -0.007030427052344105
            elif msg.servo >=  0.14902784199154218  and msg.servo <=  0.12150681902736382 :
                self.servo_pwm = (float(msg.servo) +  -0.6650470225698857 ) /  -0.006880255741044581
            elif msg.servo >=  0.12150681902736205  and msg.servo <=  0.10007398014936686 :
                self.servo_pwm = (float(msg.servo) +  -0.6859049094812355 ) /  -0.007144279625998398
            elif msg.servo >=  0.10007398014937063  and msg.servo <=  0.07768502302659419 :
                self.servo_pwm = (float(msg.servo) +  -0.7120388081719291 ) /  -0.007462985707592176
            elif msg.servo >=  0.0776850230265892  and msg.servo <=  0.05158005386633746 :
                self.servo_pwm = (float(msg.servo) +  -0.8173258159003892 ) /  -0.008701656386750588
            elif msg.servo >=  0.05158005386633807  and msg.servo <=  0.03606389736094445 :
                self.servo_pwm = (float(msg.servo) +  -0.5067206446912178 ) /  -0.0051720521684645426
            elif msg.servo >=  0.03606389736094784  and msg.servo <=  0.014916173110271824 :
                self.servo_pwm = (float(msg.servo) +  -0.6775448662981218 ) /  -0.007049241416892021
            elif msg.servo >=  0.014916173110269604  and msg.servo <=  0.0058662256044709205 :
                self.servo_pwm = (float(msg.servo) +  -0.8656112386553437 ) /  -0.00904994750579866
            elif msg.servo >=  0.005866225604469699  and msg.servo <=  -0.002539570541603531 :
                self.servo_pwm = (float(msg.servo) +  -0.4051415425429464 ) /  -0.004202898073036597
            elif msg.servo >=  -0.002539570541602254  and msg.servo <=  -0.008620303994882117 :
                self.servo_pwm = (float(msg.servo) +  -0.5872915744265398 ) /  -0.006080733453279815
            elif msg.servo >=  -0.008620303994882561  and msg.servo <=  -0.023326649667201327 :
                self.servo_pwm = (float(msg.servo) +  -0.711990633948737 ) /  -0.007353172836159383
            elif msg.servo >=  -0.023326649667201105  and msg.servo <=  -0.04649645290751481 :
                self.servo_pwm = (float(msg.servo) +  -0.7490001250099231 ) /  -0.007723267746771242
            elif msg.servo >=  -0.04649645290751636  and msg.servo <=  -0.0629780940721475 :
                self.servo_pwm = (float(msg.servo) +  -0.5193732270781519 ) /  -0.005493880388210371
            elif msg.servo >=  -0.06297809407214261  and msg.servo <=  -0.08921450963452382 :
                self.servo_pwm = (float(msg.servo) +  -0.8640419224653273 ) /  -0.008745471854127074
            elif msg.servo >=  -0.08921450963452993  and msg.servo <=  -0.10769773048520426 :
                self.servo_pwm = (float(msg.servo) +  -0.5823425146066387 ) /  -0.006161073616891455
            elif msg.servo >=  -0.10769773048520426  and msg.servo <=  -0.1295573538081054 :
                self.servo_pwm = (float(msg.servo) +  -0.7083948735697717 ) /  -0.007286541107633714
            elif msg.servo >=  -0.12955735380810252  and msg.servo <=  -0.14508716632687657 :
                self.servo_pwm = (float(msg.servo) +  -0.46575212607823646 ) /  -0.005176604172924686
            elif msg.servo >=  -0.14508716632687746  and msg.servo <=  -0.17119484962226994 :
                self.servo_pwm = (float(msg.servo) +  -0.6250894908872017 ) /  -0.006526920823848128
            elif msg.servo >=  -0.17119484962227116  and msg.servo <=  -0.19109910555053966 :
                self.servo_pwm = (float(msg.servo) +  -0.6382448914606448 ) /  -0.006634751976089475
            elif msg.servo >=  -0.19109910555053922  and msg.servo <=  -0.20946370395276015 :
                self.servo_pwm = (float(msg.servo) +  -0.5740924945420002 ) /  -0.006121532800740315
            elif msg.servo >=  -0.2094637039527596  and msg.servo <=  -0.23057544871846924 :
                self.servo_pwm = (float(msg.servo) +  -0.6913040727175193 ) /  -0.0070372482552365535
            elif msg.servo >=  -0.23057544871846863  and msg.servo <=  -0.24636522798231258 :
                self.servo_pwm = (float(msg.servo) +  -0.4589115791360478 ) /  -0.00526325975461463
            elif msg.servo >=  -0.2463652279823132  and msg.servo <=  -0.2674365568352439 :
                self.servo_pwm = (float(msg.servo) +  -0.6948207941152579 ) /  -0.007023776284310232
            elif msg.servo >=  -0.26743655683524026 :
                self.servo_pwm = (float(msg.servo) +  -0.4947438063878109 ) /  -0.005563360315496724

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
