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

        self.Steering =  self.truncate(self.Steering + 0.01*error + 0.008*self.integral ,  0.3)
        

    def truncate(self, val, bound):
        return np.maximum(-bound, np.minimum(val, bound))

class low_level_control(object):
    motor_pwm = 90
    servo_pwm = 90
    str_ang_max = 30
    str_ang_min = -40
    ecu_pub = 0
    ecu_cmd = ECU()
    sel_car = rospy.get_param("/low_level_controller/car")

    PIDController = SteeringPID(10)

    recorded_servo = [0.0]*int(20)
    fbk_servo = 0.0
    commanded_steering = 0.0
    commanded_steering_PID = 0.0
    
    def fbk_servo_callback(self,data ):
        """Unpack message from sensor, ENC"""

        self.recorded_servo.pop(0)
        self.recorded_servo.append(data.servo)
        self.value_servo = np.sum(self.recorded_servo)/len(self.recorded_servo)

        steering_out = srvOutput2Angle(self.value_servo) #-0.003530958631043808*fbk_srv.value + 1.0861319262672648
        self.PIDController.solve(float(self.commanded_steering_PID), float(steering_out))
        self.commanded_steering = self.PIDController.Steering

        if self.commanded_steering >=  0.271748477594145 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.6180603103568691 ) /  -0.0070675884237290645
        elif self.commanded_steering <=  0.2717484775941458  and self.commanded_steering >=  0.2554686004136957 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.5376531382081644 ) /  -0.005426625726816707
        elif self.commanded_steering <=  0.2554686004136967  and self.commanded_steering >=  0.22637315820481485 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.759789598700981 ) /  -0.00969848073629393
        elif self.commanded_steering <=  0.22637315820481269  and self.commanded_steering >=  0.20571227696341288 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.60515598096381 ) /  -0.00688696041379995
        elif self.commanded_steering <=  0.2057122769634162  and self.commanded_steering >=  0.16617707955839722 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.7789726393361914 ) /  -0.009883799351254744
        elif self.commanded_steering <=  0.16617707955839633  and self.commanded_steering >=  0.14552846784025059 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.5929150550667419 ) /  -0.006882870572715251
        elif self.commanded_steering <=  0.14552846784024898  and self.commanded_steering >=  0.11796964644491936 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.7426362647390593 ) /  -0.009186273798443235
        elif self.commanded_steering <=  0.11796964644491859  and self.commanded_steering >=  0.09945597233128706 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.5376129263538998 ) /  -0.006171224704543841
        elif self.commanded_steering <=  0.099455972331286  and self.commanded_steering >=  0.07034295529014778 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.7884640423048922 ) /  -0.009704339013712763
        elif self.commanded_steering <=  0.07034295529014867  and self.commanded_steering >=  0.044084479382825226 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.7180520276707935 ) /  -0.008752825302441146
        elif self.commanded_steering <=  0.04408447938282428  and self.commanded_steering >=  0.0330607836356801 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.4684967656478735 ) /  -0.005511847873572068
        elif self.commanded_steering <=  0.033060783635680435  and self.commanded_steering >=  0.007520301524546746 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.7056268125622019 ) /  -0.008513494037044576
        elif self.commanded_steering <=  0.0075203015245473015  and self.commanded_steering >=  -0.0015686544265943736 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.7528146895181625 ) /  -0.009088955951141649
        elif self.commanded_steering <=  -0.0015686544265950397  and self.commanded_steering >=  -0.010379109894857463 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.7296991494391903 ) /  -0.008810455468262474
        elif self.commanded_steering <=  -0.010379109894858463  and self.commanded_steering >=  -0.01670257042834955 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.5207915749183991 ) /  -0.006323460533491161
        elif self.commanded_steering <=  -0.016702570428348218  and self.commanded_steering >=  -0.026038642972046833 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.7768635957860361 ) /  -0.009336072543698639
        elif self.commanded_steering <=  -0.026038642972047055  and self.commanded_steering >=  -0.04154722815221579 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.6408305197752049 ) /  -0.0077542925900843255
        elif self.commanded_steering <=  -0.04154722815221584  and self.commanded_steering >=  -0.06449664329690569 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.4633399050309606 ) /  -0.00573735378617246
        elif self.commanded_steering <=  -0.06449664329690519  and self.commanded_steering >=  -0.09067831040253727 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.7384078146091428 ) /  -0.008727222368544
        elif self.commanded_steering <=  -0.09067831040253777  and self.commanded_steering >=  -0.10722629235046827 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.43334111794859426 ) /  -0.0055159939826434955
        elif self.commanded_steering <=  -0.10722629235046754  and self.commanded_steering >=  -0.12902829503324786 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.6049724619536923 ) /  -0.007267334227593467
        elif self.commanded_steering <=  -0.1290282950332492  and self.commanded_steering >=  -0.15277998557153327 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.6706119530889807 ) /  -0.007917230179428018
        elif self.commanded_steering <=  -0.1527799855715316  and self.commanded_steering >=  -0.17309500293432833 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.5514739496720873 ) /  -0.006771672454265566
        elif self.commanded_steering <=  -0.17309500293433233  and self.commanded_steering >=  -0.19516681168236716 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.6141328424122462 ) /  -0.007357269582678304
        elif self.commanded_steering <=  -0.1951668116823626  and self.commanded_steering >=  -0.21082415402432397 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.6659870171255149 ) /  -0.007828671170980704
        elif self.commanded_steering <=  -0.2108241540243243  and self.commanded_steering >=  -0.23687110133349532 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.7615952121847259 ) /  -0.008682315769723663
        elif self.commanded_steering <=  -0.2368711013334988  and self.commanded_steering >=  -0.251275282681168 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.31528918366048914 ) /  -0.0048013937825564165
        elif self.commanded_steering <=  -0.2512752826811647  and self.commanded_steering >=  -0.2734474105207739 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.6208284123434624 ) /  -0.007390709279869722
        elif self.commanded_steering <=  -0.2734474105207766 :
            self.servo_pwm = (float(self.commanded_steering) +  -0.47700527437619844 ) /  -0.006202088304933678
        
        # self.update_arduino()



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
        self.commanded_steering_PID = msg.servo

        msg.servo = self.commanded_steering

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
            if msg.servo >=  0.271748477594145 :
                self.servo_pwm = (float(msg.servo) +  -0.6180603103568691 ) /  -0.0070675884237290645
            elif msg.servo <=  0.2717484775941458  and msg.servo >=  0.2554686004136957 :
                self.servo_pwm = (float(msg.servo) +  -0.5376531382081644 ) /  -0.005426625726816707
            elif msg.servo <=  0.2554686004136967  and msg.servo >=  0.22637315820481485 :
                self.servo_pwm = (float(msg.servo) +  -0.759789598700981 ) /  -0.00969848073629393
            elif msg.servo <=  0.22637315820481269  and msg.servo >=  0.20571227696341288 :
                self.servo_pwm = (float(msg.servo) +  -0.60515598096381 ) /  -0.00688696041379995
            elif msg.servo <=  0.2057122769634162  and msg.servo >=  0.16617707955839722 :
                self.servo_pwm = (float(msg.servo) +  -0.7789726393361914 ) /  -0.009883799351254744
            elif msg.servo <=  0.16617707955839633  and msg.servo >=  0.14552846784025059 :
                self.servo_pwm = (float(msg.servo) +  -0.5929150550667419 ) /  -0.006882870572715251
            elif msg.servo <=  0.14552846784024898  and msg.servo >=  0.11796964644491936 :
                self.servo_pwm = (float(msg.servo) +  -0.7426362647390593 ) /  -0.009186273798443235
            elif msg.servo <=  0.11796964644491859  and msg.servo >=  0.09945597233128706 :
                self.servo_pwm = (float(msg.servo) +  -0.5376129263538998 ) /  -0.006171224704543841
            elif msg.servo <=  0.099455972331286  and msg.servo >=  0.07034295529014778 :
                self.servo_pwm = (float(msg.servo) +  -0.7884640423048922 ) /  -0.009704339013712763
            elif msg.servo <=  0.07034295529014867  and msg.servo >=  0.044084479382825226 :
                self.servo_pwm = (float(msg.servo) +  -0.7180520276707935 ) /  -0.008752825302441146
            elif msg.servo <=  0.04408447938282428  and msg.servo >=  0.0330607836356801 :
                self.servo_pwm = (float(msg.servo) +  -0.4684967656478735 ) /  -0.005511847873572068
            elif msg.servo <=  0.033060783635680435  and msg.servo >=  0.007520301524546746 :
                self.servo_pwm = (float(msg.servo) +  -0.7056268125622019 ) /  -0.008513494037044576
            elif msg.servo <=  0.0075203015245473015  and msg.servo >=  -0.0015686544265943736 :
                self.servo_pwm = (float(msg.servo) +  -0.7528146895181625 ) /  -0.009088955951141649
            elif msg.servo <=  -0.0015686544265950397  and msg.servo >=  -0.010379109894857463 :
                self.servo_pwm = (float(msg.servo) +  -0.7296991494391903 ) /  -0.008810455468262474
            elif msg.servo <=  -0.010379109894858463  and msg.servo >=  -0.01670257042834955 :
                self.servo_pwm = (float(msg.servo) +  -0.5207915749183991 ) /  -0.006323460533491161
            elif msg.servo <=  -0.016702570428348218  and msg.servo >=  -0.026038642972046833 :
                self.servo_pwm = (float(msg.servo) +  -0.7768635957860361 ) /  -0.009336072543698639
            elif msg.servo <=  -0.026038642972047055  and msg.servo >=  -0.04154722815221579 :
                self.servo_pwm = (float(msg.servo) +  -0.6408305197752049 ) /  -0.0077542925900843255
            elif msg.servo <=  -0.04154722815221584  and msg.servo >=  -0.06449664329690569 :
                self.servo_pwm = (float(msg.servo) +  -0.4633399050309606 ) /  -0.00573735378617246
            elif msg.servo <=  -0.06449664329690519  and msg.servo >=  -0.09067831040253727 :
                self.servo_pwm = (float(msg.servo) +  -0.7384078146091428 ) /  -0.008727222368544
            elif msg.servo <=  -0.09067831040253777  and msg.servo >=  -0.10722629235046827 :
                self.servo_pwm = (float(msg.servo) +  -0.43334111794859426 ) /  -0.0055159939826434955
            elif msg.servo <=  -0.10722629235046754  and msg.servo >=  -0.12902829503324786 :
                self.servo_pwm = (float(msg.servo) +  -0.6049724619536923 ) /  -0.007267334227593467
            elif msg.servo <=  -0.1290282950332492  and msg.servo >=  -0.15277998557153327 :
                self.servo_pwm = (float(msg.servo) +  -0.6706119530889807 ) /  -0.007917230179428018
            elif msg.servo <=  -0.1527799855715316  and msg.servo >=  -0.17309500293432833 :
                self.servo_pwm = (float(msg.servo) +  -0.5514739496720873 ) /  -0.006771672454265566
            elif msg.servo <=  -0.17309500293433233  and msg.servo >=  -0.19516681168236716 :
                self.servo_pwm = (float(msg.servo) +  -0.6141328424122462 ) /  -0.007357269582678304
            elif msg.servo <=  -0.1951668116823626  and msg.servo >=  -0.21082415402432397 :
                self.servo_pwm = (float(msg.servo) +  -0.6659870171255149 ) /  -0.007828671170980704
            elif msg.servo <=  -0.2108241540243243  and msg.servo >=  -0.23687110133349532 :
                self.servo_pwm = (float(msg.servo) +  -0.7615952121847259 ) /  -0.008682315769723663
            elif msg.servo <=  -0.2368711013334988  and msg.servo >=  -0.251275282681168 :
                self.servo_pwm = (float(msg.servo) +  -0.31528918366048914 ) /  -0.0048013937825564165
            elif msg.servo <=  -0.2512752826811647  and msg.servo >=  -0.2734474105207739 :
                self.servo_pwm = (float(msg.servo) +  -0.6208284123434624 ) /  -0.007390709279869722
            elif msg.servo <=  -0.2734474105207766 :
                self.servo_pwm = (float(msg.servo) +  -0.47700527437619844 ) /  -0.006202088304933678



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
