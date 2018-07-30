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
            if msg.servo >=  0.25101073177496236 :
                self.servo_pwm = (float(msg.servo) +  -0.6523243920226947 ) /  -0.0072966120045042234
            elif msg.servo <=  0.25101073177496636  and msg.servo >=  0.2314058363891589 :
                self.servo_pwm = (float(msg.servo) +  -0.6104338138481035 ) /  -0.006534965128602493
            elif msg.servo <=  0.23140583638915363  and msg.servo >=  0.20987486744762235 :
                self.servo_pwm = (float(msg.servo) +  -0.543604886041357 ) /  -0.005382742235382818
            elif msg.servo <=  0.20987486744762474  and msg.servo >=  0.18040649551999455 :
                self.servo_pwm = (float(msg.servo) +  -0.8188878872853146 ) /  -0.009822790642543384
            elif msg.servo <=  0.18040649551999022  and msg.servo >=  0.15755732136543577 :
                self.servo_pwm = (float(msg.servo) +  -0.6754719355353359 ) /  -0.007616391384851472
            elif msg.servo <=  0.15755732136544343  and msg.servo >=  0.13513720545829078 :
                self.servo_pwm = (float(msg.servo) +  -0.6657466152609044 ) /  -0.007473371969050896
            elif msg.servo <=  0.13513720545828767  and msg.servo >=  0.10725378496736959 :
                self.servo_pwm = (float(msg.servo) +  -0.7950448237433487 ) /  -0.009294473496972691
            elif msg.servo <=  0.10725378496736937  and msg.servo >=  0.08572935110935054 :
                self.servo_pwm = (float(msg.servo) +  -0.6381898201318317 ) /  -0.007174811286006248
            elif msg.servo <=  0.08572935110934932  and msg.servo >=  0.06382835221027816 :
                self.servo_pwm = (float(msg.servo) +  -0.647854989518842 ) /  -0.007300332966357048
            elif msg.servo <=  0.06382835221027916  and msg.servo >=  0.03931265941845419 :
                self.servo_pwm = (float(msg.servo) +  -0.7175801599922763 ) /  -0.008171897597274965
            elif msg.servo <=  0.03931265941845341  and msg.servo >=  0.017708566164701067 :
                self.servo_pwm = (float(msg.servo) +  -0.6370259061056011 ) /  -0.007201364417917441
            elif msg.servo <=  0.017708566164702844  and msg.servo >=  0.006036701440005454 :
                self.servo_pwm = (float(msg.servo) +  -0.519598749326691 ) /  -0.0058359323623487
            elif msg.servo <=  0.006036701440003789  and msg.servo >=  0.00015487366844557648 :
                self.servo_pwm = (float(msg.servo) +  -0.5236375453371245 ) /  -0.00588182777155819
            elif msg.servo <=  0.00015487366844546546  and msg.servo >=  -0.010405592614529113 :
                self.servo_pwm = (float(msg.servo) +  -0.9400363728531845 ) /  -0.010560466282974596
            elif msg.servo <=  -0.010405592614527559  and msg.servo >=  -0.023525084085319947 :
                self.servo_pwm = (float(msg.servo) +  -0.5799715235711307 ) /  -0.006559745735396203
            elif msg.servo <=  -0.023525084085320835  and msg.servo >=  -0.04090645768596346 :
                self.servo_pwm = (float(msg.servo) +  -0.5095037063343894 ) /  -0.0057937912002142405
            elif msg.servo <=  -0.04090645768596424  and msg.servo >=  -0.059341934182953526 :
                self.servo_pwm = (float(msg.servo) +  -0.5428836313853643 ) /  -0.006145158832329774
            elif msg.servo <=  -0.05934193418295319  and msg.servo >=  -0.07892108877267956 :
                self.servo_pwm = (float(msg.servo) +  -0.5802437824147734 ) /  -0.006526384863242108
            elif msg.servo <=  -0.07892108877267945  and msg.servo >=  -0.10429297192570863 :
                self.servo_pwm = (float(msg.servo) +  -0.775265644045968 ) /  -0.008457294384343044
            elif msg.servo <=  -0.10429297192570874  and msg.servo >=  -0.1273614764160773 :
                self.servo_pwm = (float(msg.servo) +  -0.6954151837403996 ) /  -0.007689501496789503
            elif msg.servo <=  -0.1273614764160773  and msg.servo >=  -0.14760746289270177 :
                self.servo_pwm = (float(msg.servo) +  -0.594745374583529 ) /  -0.0067486621588748255
            elif msg.servo <=  -0.147607462892704  and msg.servo >=  -0.16592955942563692 :
                self.servo_pwm = (float(msg.servo) +  -0.8601078464186098 ) /  -0.00916104826646649
            elif msg.servo <=  -0.16592955942563659  and msg.servo >=  -0.1857666259072659 :
                self.servo_pwm = (float(msg.servo) +  -0.5746542558885244 ) /  -0.0066123554938764375
            elif msg.servo <=  -0.185766625907263  and msg.servo >=  -0.20401013544697888 :
                self.servo_pwm = (float(msg.servo) +  -0.513567906448514 ) /  -0.006081169846571974
            elif msg.servo <=  -0.2040101354469821  and msg.servo >=  -0.22493867244578258 :
                self.servo_pwm = (float(msg.servo) +  -0.6191789865058376 ) /  -0.006976178999600167
            elif msg.servo <=  -0.22493867244577548  and msg.servo >=  -0.24201984478136818 :
                self.servo_pwm = (float(msg.servo) +  -0.464001945089797 ) /  -0.005693724111864235
            elif msg.servo <=  -0.24201984478137384  and msg.servo >=  -0.2589552247315977 :
                self.servo_pwm = (float(msg.servo) +  -0.4579758598278809 ) /  -0.005645126650074635
            elif msg.servo <=  -0.2589552247315916 :
                self.servo_pwm = (float(msg.servo) +  -1.017130952562731 ) /  -0.010047922655860807




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
