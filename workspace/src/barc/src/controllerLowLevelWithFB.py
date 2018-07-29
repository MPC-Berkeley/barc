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

def srvOutput2Angle(fbk_srv):
    angle_rad =  -0.003530958631043808 *fbk_srv +  1.0861319262672648
    return angle_rad


class low_level_control(object):
    motor_pwm = 90
    servo_pwm = 90
    str_ang_max = 30
    str_ang_min = -40
    ecu_pub = 0
    ecu_cmd = ECU()
    sel_car = rospy.get_param("/low_level_controller/car")
    pid_active = False

    recorded_servo = [0.0]*int(20)
    fbk_servo = 0.0
    commanded_steering = 0.0
    
    def fbk_servo_callback(self,data ):
        """Unpack message from sensor, ENC"""


        if self.pid_active == True:
            self.recorded_servo.pop(0)
            self.recorded_servo.append(data.servo)
            self.value_servo = np.sum(self.recorded_servo)/len(self.recorded_servo)

            steering_out = srvOutput2Angle(self.value_servo) #-0.003530958631043808*fbk_srv.value + 1.0861319262672648
            PIDController.solve(float(self.commanded_steering), float(steering_out))
            self.commanded_steering = PIDController.Steering

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
        if self.pid_active == False:
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
            if msg.servo >=  0.2737151266162874 :
                self.servo_pwm = (float(msg.servo) +  -0.5572243439015763 ) /  -0.004888089953194636
            elif msg.servo <=  0.2737151266162857  and msg.servo >=  0.24588579418222145 :
                self.servo_pwm = (float(msg.servo) +  -0.6772404469102168 ) /  -0.006957333108516053
            elif msg.servo <=  0.24588579418222578  and msg.servo >=  0.2219399489877507 :
                self.servo_pwm = (float(msg.servo) +  -0.7407665948680447 ) /  -0.00798194839815837
            elif msg.servo <=  0.22193994898774821  and msg.servo >=  0.20456794113693538 :
                self.servo_pwm = (float(msg.servo) +  -0.5983334524220261 ) /  -0.005790669283604275
            elif msg.servo <=  0.20456794113693488  and msg.servo >=  0.16964630515811052 :
                self.servo_pwm = (float(msg.servo) +  -0.798235752776948 ) /  -0.008730408994706076
            elif msg.servo <=  0.16964630515810902  and msg.servo >=  0.1490210383016648 :
                self.servo_pwm = (float(msg.servo) +  -0.6646527097127707 ) /  -0.006875088952148079
            elif msg.servo <=  0.14902103830166946  and msg.servo >=  0.12172409763421688 :
                self.servo_pwm = (float(msg.servo) +  -0.8314445549879841 ) /  -0.009098980222484196
            elif msg.servo <=  0.12172409763421244  and msg.servo >=  0.08764356307657761 :
                self.servo_pwm = (float(msg.servo) +  -0.7862945215080924 ) /  -0.008520133639408718
            elif msg.servo <=  0.08764356307657784  and msg.servo >=  0.06416555371563215 :
                self.servo_pwm = (float(msg.servo) +  -0.7293758189424275 ) /  -0.00782600312031524
            elif msg.servo <=  0.06416555371563343  and msg.servo >=  0.049026144027575824 :
                self.servo_pwm = (float(msg.servo) +  -0.4931154948772664 ) /  -0.0050464698960192116
            elif msg.servo <=  0.04902614402757677  and msg.servo >=  0.019346977704423174 :
                self.servo_pwm = (float(msg.servo) +  -0.9196150228400841 ) /  -0.00989305544105122
            elif msg.servo <=  0.019346977704422674  and msg.servo >=  0.004899257293035975 :
                self.servo_pwm = (float(msg.servo) +  -0.4575944968498187 ) /  -0.0048159068037955605
            elif msg.servo <=  0.004899257293036197  and msg.servo >=  -0.003670419800220648 :
                self.servo_pwm = (float(msg.servo) +  -0.8104489040591819 ) /  -0.008569677093256869
            elif msg.servo <=  -0.003670419800221314  and msg.servo >=  -0.01234066254154964 :
                self.servo_pwm = (float(msg.servo) +  -0.40816611041287204 ) /  -0.004335121370664141
            elif msg.servo <=  -0.012340662541550196  and msg.servo >=  -0.033467514050333436 :
                self.servo_pwm = (float(msg.servo) +  -0.6707608695757744 ) /  -0.0070422838362610785
            elif msg.servo <=  -0.03346751405033399  and msg.servo >=  -0.056823527000686824 :
                self.servo_pwm = (float(msg.servo) +  -0.7450662509614252 ) /  -0.007785337650117592
            elif msg.servo <=  -0.056823527000686824  and msg.servo >=  -0.08061750843612614 :
                self.servo_pwm = (float(msg.servo) +  -0.7601031689493961 ) /  -0.007931327145146437
            elif msg.servo <=  -0.0806175084361267  and msg.servo >=  -0.10767204879168601 :
                self.servo_pwm = (float(msg.servo) +  -0.8753095841269696 ) /  -0.009018180118519776
            elif msg.servo <=  -0.10767204879168418  and msg.servo >=  -0.12247133674351779 :
                self.servo_pwm = (float(msg.servo) +  -0.43003541345826973 ) /  -0.004933095983944531
            elif msg.servo <=  -0.12247133674351507  and msg.servo >=  -0.14836988233154302 :
                self.servo_pwm = (float(msg.servo) +  -0.8444076985428655 ) /  -0.008632848529342683
            elif msg.servo <=  -0.14836988233155257  and msg.servo >=  -0.16948230816771503 :
                self.servo_pwm = (float(msg.servo) +  -0.6609397747213391 ) /  -0.007037475278720798
            elif msg.servo <=  -0.1694823081677087  and msg.servo >=  -0.19037981680303118 :
                self.servo_pwm = (float(msg.servo) +  -0.6524863648216402 ) /  -0.006965836211774144
            elif msg.servo <=  -0.19037981680302718  and msg.servo >=  -0.21107656694948573 :
                self.servo_pwm = (float(msg.servo) +  -0.6443891057708028 ) /  -0.006898916715486198
            elif msg.servo <=  -0.2110765669494863  and msg.servo >=  -0.23182799017505207 :
                self.servo_pwm = (float(msg.servo) +  -0.646648926373901 ) /  -0.006917141075188607
            elif msg.servo <=  -0.23182799017505556  and msg.servo >=  -0.24638969449951093 :
                self.servo_pwm = (float(msg.servo) +  -0.384617492893558 ) /  -0.004853901441485146
            elif msg.servo <=  -0.24638969449950787  and msg.servo >=  -0.2703203662614284 :
                self.servo_pwm = (float(msg.servo) +  -0.7906060818503868 ) /  -0.007976890587306882
            elif msg.servo <=  -0.2703203662614305 :
                self.servo_pwm = (float(msg.servo) +  -0.6740515990814404 ) /  -0.007100541092803541



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

class SteeringPID(object):

    def __init__(self, loop_rate):

        self.Ts       = 1/loop_rate
        self.Steering = 0.0
        self.integral = 0.0

    def solve(self, comanded_delta, measured_delta):
        """Computes control action
        Arguments:
            delta: current steering angle
        """
        error = comanded_delta - measured_delta
        #print error, "\n"
        self.integral = self.integral +  error*self.Ts
        self.integral = self.truncate( self.integral , 3 )

        self.Steering =  self.truncate(self.Steering + 0.02*error + 0.006*self.integral ,  0.3)
        

    def truncate(self, val, bound):
        return np.maximum(-bound, np.minimum(val, bound))
        
#############################################################
if __name__ == '__main__':
    try:
        arduino_interface()
    except ROSInterruptException:
        pass
