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
    # angle_rad =  -0.003530958631043808 *fbk_srv +  1.0861319262672648
    angle_rad =  -0.003325686873493677 *fbk_srv +  1.1045130391911997
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
            if msg.servo <=  -0.28427866720276546 :
                self.servo_pwm = (float(msg.servo) +  0.7667652891218538 ) /  0.008464677577527866
            elif msg.servo >=  -0.2842786672027673  and msg.servo <=  -0.26318686746463726 :
                self.servo_pwm = (float(msg.servo) +  0.8853949597394727 ) /  0.010545899869065007
            elif msg.servo >=  -0.26318686746463116  and msg.servo <=  -0.23170159114638744 :
                self.servo_pwm = (float(msg.servo) +  0.8823973017234237 ) /  0.01049509210608123
            elif msg.servo >=  -0.2317015911463911  and msg.servo <=  -0.20405523567075 :
                self.servo_pwm = (float(msg.servo) +  0.8030596043096409 ) /  0.009215451825213706
            elif msg.servo >=  -0.2040552356707508  and msg.servo <=  -0.17535113759800158 :
                self.servo_pwm = (float(msg.servo) +  0.8259773605803177 ) /  0.009568032690916413
            elif msg.servo >=  -0.1753511375980017  and msg.servo <=  -0.14467330043109927 :
                self.servo_pwm = (float(msg.servo) +  0.8707154467144568 ) /  0.01022594572230081
            elif msg.servo >=  -0.14467330043110038  and msg.servo <=  -0.10535383771285411 :
                self.servo_pwm = (float(msg.servo) +  1.0752339180962618 ) /  0.013106487572748753
            elif msg.servo >=  -0.1053538377128499  and msg.servo <=  -0.07256813845887589 :
                self.servo_pwm = (float(msg.servo) +  0.9140677526442104 ) /  0.010928566417991357
            elif msg.servo >=  -0.07256813845887311  and msg.servo <=  -0.03838787670736998 :
                self.servo_pwm = (float(msg.servo) +  0.9498615234141183 ) /  0.011393420583834354
            elif msg.servo >=  -0.03838787670737587  and msg.servo <=  -0.016377541782759297 :
                self.servo_pwm = (float(msg.servo) +  0.9188012736920392 ) /  0.011005167462308292
            elif msg.servo >=  -0.016377541782754523  and msg.servo <=  -0.0008380427195389117 :
                self.servo_pwm = (float(msg.servo) +  1.2906164649664262 ) /  0.01553949906321551
            elif msg.servo >=  -0.0008380427195404105  and msg.servo <=  0.0030116995206450614 :
                self.servo_pwm = (float(msg.servo) +  0.32036664865493625 ) /  0.003849742240185492
            elif msg.servo >=  0.003011699520642064  and msg.servo <=  0.03087814781653364 :
                self.servo_pwm = (float(msg.servo) +  0.7772488527643225 ) /  0.009288816098630531
            elif msg.servo >=  0.030878147816537305  and msg.servo <=  0.06526559598978976 :
                self.servo_pwm = (float(msg.servo) +  0.9663578492077834 ) /  0.01146248272441748
            elif msg.servo >=  0.06526559598979342  and msg.servo <=  0.1044716917844073 :
                self.servo_pwm = (float(msg.servo) +  1.1109172778486207 ) /  0.013068698598204602
            elif msg.servo >=  0.10447169178440507  and msg.servo <=  0.13820382448062407 :
                self.servo_pwm = (float(msg.servo) +  0.9412244217983843 ) /  0.011244044232073003
            elif msg.servo >=  0.1382038244806253  and msg.servo <=  0.1703134232510487 :
                self.servo_pwm = (float(msg.servo) +  0.8893033361729284 ) /  0.010703199590141184
            elif msg.servo >=  0.17031342325104348  and msg.servo <=  0.20929975927400823 :
                self.servo_pwm = (float(msg.servo) +  1.116235665506792 ) /  0.012995445340988237
            elif msg.servo >=  0.209299759274016  and msg.servo <=  0.24595097822757306 :
                self.servo_pwm = (float(msg.servo) +  1.0368416851469224 ) /  0.012217072984519005
            elif msg.servo >=  0.24595097822756506  and msg.servo <=  0.2739384641169089 :
                self.servo_pwm = (float(msg.servo) +  0.733611027899473 ) /  0.009329161963114648
            elif msg.servo >=  0.27393846411691636 :
                self.servo_pwm = (float(msg.servo) +  0.5560596126520972 ) /  0.007685167377490866



            # self.servo_pwm = 81.4 + 89.1*float(msg.servo)
        elif self.sel_car == "NewBARC":
            # if msg.servo >=  0.2528518882484856 :
            #     self.servo_pwm = (float(msg.servo) +  -0.7840318925472398 ) /  -0.009657818259977349
            # elif msg.servo <=  0.25285188824849125  and msg.servo >=  0.23924549953847157 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5023023479321851 ) /  -0.004535462903339889
            # elif msg.servo <=  0.23924549953846597  and msg.servo >=  0.21759911999993914 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6577421706166513 ) /  -0.007215459846175609
            # elif msg.servo <=  0.21759911999993775  and msg.servo >=  0.19226870246068084 :
            #     self.servo_pwm = (float(msg.servo) +  -0.7326509432981622 ) /  -0.008443472513085646
            # elif msg.servo <=  0.19226870246068745  and msg.servo >=  0.16921629385936743 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6840534192888468 ) /  -0.0076841362004399895
            # elif msg.servo <=  0.1692162938593671  and msg.servo >=  0.1426687738594652 :
            #     self.servo_pwm = (float(msg.servo) +  -0.7621109071905099 ) /  -0.008849173333300639
            # elif msg.servo <=  0.14266877385945853  and msg.servo >=  0.11917209610442558 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6909245881435615 ) /  -0.007832225918344328
            # elif msg.servo <=  0.11917209610443313  and msg.servo >=  0.09641265692896728 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6729851160407695 ) /  -0.007586479725155293
            # elif msg.servo <=  0.09641265692896472  and msg.servo >=  0.07174689309441895 :
            #     self.servo_pwm = (float(msg.servo) +  -0.7212786740707922 ) /  -0.00822192127818194
            # elif msg.servo <=  0.07174689309441751  and msg.servo >=  0.04871659287825236 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6782114654534332 ) /  -0.007676766738721718
            # elif msg.servo <=  0.04871659287825292  and msg.servo >=  0.024470329046924477 :
            #     self.servo_pwm = (float(msg.servo) +  -0.7114478042678961 ) /  -0.008082087943776137
            # elif msg.servo <=  0.02447032904692603  and msg.servo >=  0.011819154621268813 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5621452421373546 ) /  -0.006325587212828572
            # elif msg.servo <=  0.011819154621267036  and msg.servo >=  -0.0026132690535047853 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6396295844738418 ) /  -0.007216211837385916
            # elif msg.servo <=  -0.002613269053503564  and msg.servo >=  -0.008983638296737095 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5643495935942784 ) /  -0.006370369243233505
            # elif msg.servo <=  -0.008983638296738206  and msg.servo >=  -0.0181697767778225 :
            #     self.servo_pwm = (float(msg.servo) +  -0.8177688250008434 ) /  -0.00918613848108424
            # elif msg.servo <=  -0.01816977677781989  and msg.servo >=  -0.03356855522715041 :
            #     self.servo_pwm = (float(msg.servo) +  -0.448926502851873 ) /  -0.005132926149776845
            # elif msg.servo <=  -0.03356855522715141  and msg.servo >=  -0.05293285987344298 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5731796570233179 ) /  -0.006454768215430524
            # elif msg.servo <=  -0.05293285987344376  and msg.servo >=  -0.07090564379317066 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5281871535310595 ) /  -0.005990927973242302
            # elif msg.servo <=  -0.0709056437931721  and msg.servo >=  -0.0976633069038082 :
            #     self.servo_pwm = (float(msg.servo) +  -0.821016459894697 ) /  -0.00891922103687869
            # elif msg.servo <=  -0.09766330690380731  and msg.servo >=  -0.12010407363044129 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6728030173772919 ) /  -0.007480255575544653
            # elif msg.servo <=  -0.12010407363044195  and msg.servo >=  -0.1455125143306717 :
            #     self.servo_pwm = (float(msg.servo) +  -0.7776608311110058 ) /  -0.008469480233409885
            # elif msg.servo <=  -0.14551251433067408  and msg.servo >=  -0.16206433542062265 :
            #     self.servo_pwm = (float(msg.servo) +  -0.4558703186041246 ) /  -0.005517273696649529
            # elif msg.servo <=  -0.1620643354206166  and msg.servo >=  -0.1882113020430658 :
            #     self.servo_pwm = (float(msg.servo) +  -0.8140890851508209 ) /  -0.008715655540816407
            # elif msg.servo <=  -0.18821130204307046  and msg.servo >=  -0.20657905577882718 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5158859244942703 ) /  -0.006122584578585572
            # elif msg.servo <=  -0.20657905577882774  and msg.servo >=  -0.22923425472603742 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6845254361447545 ) /  -0.007551732982403239
            # elif msg.servo <=  -0.2292342547260357  and msg.servo >=  -0.24661030999398664 :
            #     self.servo_pwm = (float(msg.servo) +  -0.47159997441465124 ) /  -0.005792018422650306
            # elif msg.servo <=  -0.24661030999398825  and msg.servo >=  -0.2691403717199723 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6846322413466865 ) /  -0.007510020575328022
            # elif msg.servo <=  -0.26914037171996885 :
            #     self.servo_pwm = (float(msg.servo) +  -0.32193278464528574 ) /  -0.004654119341458697
            if msg.servo >=  0.2566846775465117 :
                self.servo_pwm = (float(msg.servo) +  -0.6772137645871529 ) /  -0.0072505015007007095
            elif msg.servo <=  0.25668467754650887  and msg.servo >=  0.23146756582979477 :
                self.servo_pwm = (float(msg.servo) +  -0.6223327974388628 ) /  -0.006304277929178517
            elif msg.servo <=  0.23146756582980016  and msg.servo >=  0.20829424369308602 :
                self.servo_pwm = (float(msg.servo) +  -0.7103828899885596 ) /  -0.007724440712238055
            elif msg.servo <=  0.2082942436930889  and msg.servo >=  0.18468046651630532 :
                self.servo_pwm = (float(msg.servo) +  -0.7199260825233992 ) /  -0.00787125905892785
            elif msg.servo <=  0.18468046651629766  and msg.servo >=  0.1529475905995923 :
                self.servo_pwm = (float(msg.servo) +  -0.7241393571002901 ) /  -0.007933218979176359
            elif msg.servo <=  0.15294759059959273  and msg.servo >=  0.12628127241126386 :
                self.servo_pwm = (float(msg.servo) +  -0.792939227119488 ) /  -0.00888877272944299
            elif msg.servo <=  0.12628127241126985  and msg.servo >=  0.10497656163730706 :
                self.servo_pwm = (float(msg.servo) +  -0.6588990417603376 ) /  -0.007101570257987571
            elif msg.servo <=  0.10497656163730629  and msg.servo >=  0.08214429016307656 :
                self.servo_pwm = (float(msg.servo) +  -0.6986156199672782 ) /  -0.007610757158076563
            elif msg.servo <=  0.08214429016307923  and msg.servo >=  0.05702763786489473 :
                self.servo_pwm = (float(msg.servo) +  -0.7602939022140623 ) /  -0.008372217432728186
            elif msg.servo <=  0.05702763786489684  and msg.servo >=  0.036869070854447794 :
                self.servo_pwm = (float(msg.servo) +  -0.6214675141574701 ) /  -0.0067195223368163495
            elif msg.servo <=  0.03686907085444058  and msg.servo >=  0.0078028366744673505 :
                self.servo_pwm = (float(msg.servo) +  -0.8797898620736663 ) /  -0.009688744726657766
            elif msg.servo <=  0.007802836674472138  and msg.servo >=  0.007246066618238282 :
                self.servo_pwm = (float(msg.servo) +  -0.05791214173551918 ) /  -0.0005567700562338561
            elif msg.servo <=  0.0072460666182367905  and msg.servo >=  0.0010465538152072407 :
                self.servo_pwm = (float(msg.servo) +  -0.5714017316939339 ) /  -0.006199512803029638
            elif msg.servo <=  0.0010465538152094611  and msg.servo >=  -0.010414027483334864 :
                self.servo_pwm = (float(msg.servo) +  -0.5282332935482473 ) /  -0.005730290649272151
            elif msg.servo <=  -0.010414027483335975  and msg.servo >=  -0.03287794221959617 :
                self.servo_pwm = (float(msg.servo) +  -0.6934553009194863 ) /  -0.0074879715787534275
            elif msg.servo <=  -0.03287794221959872  and msg.servo >=  -0.05696110792723241 :
                self.servo_pwm = (float(msg.servo) +  -0.7458110823272258 ) /  -0.008027721902544582
            elif msg.servo <=  -0.05696110792722808  and msg.servo >=  -0.07419607614956403 :
                self.servo_pwm = (float(msg.servo) +  -0.5175378328173046 ) /  -0.0057449894074453264
            elif msg.servo <=  -0.07419607614956558  and msg.servo >=  -0.10402343878170728 :
                self.servo_pwm = (float(msg.servo) +  -0.9498767075539637 ) /  -0.009942454210713877
            elif msg.servo <=  -0.1040234387817085  and msg.servo >=  -0.12676022263258424 :
                self.servo_pwm = (float(msg.servo) +  -0.6993429239492363 ) /  -0.007578927950291931
            elif msg.servo <=  -0.12676022263258357  and msg.servo >=  -0.14476343130612246 :
                self.servo_pwm = (float(msg.servo) +  -0.5273563591726613 ) /  -0.006001069557846283
            elif msg.servo <=  -0.14476343130612335  and msg.servo >=  -0.16352724485132175 :
                self.servo_pwm = (float(msg.servo) +  -0.5557522743812868 ) /  -0.006254604515066162
            elif msg.servo <=  -0.16352724485132186  and msg.servo >=  -0.1874521557128075 :
                self.servo_pwm = (float(msg.servo) +  -0.7535943381722943 ) /  -0.007974970287161879
            elif msg.servo <=  -0.18745215571281282  and msg.servo >=  -0.20082531326732567 :
                self.servo_pwm = (float(msg.servo) +  -0.601564140003442 ) /  -0.006686578777256398
            elif msg.servo <=  -0.2008253132673209  and msg.servo >=  -0.2079742960520491 :
                self.servo_pwm = (float(msg.servo) +  -0.65705262090007 ) /  -0.007148982784728257
            elif msg.servo <=  -0.20797429605204176  and msg.servo >=  -0.2291596853957738 :
                self.servo_pwm = (float(msg.servo) +  -0.6465030741451508 ) /  -0.007061796447910683
            elif msg.servo <=  -0.22915968539578335  and msg.servo >=  -0.25069948491193395 :
                self.servo_pwm = (float(msg.servo) +  -0.6611520279384434 ) /  -0.0071799331720502155
            elif msg.servo <=  -0.2506994849119306 :
                self.servo_pwm = (float(msg.servo) +  -0.37048861145868683 ) /  -0.004891244853311948







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
