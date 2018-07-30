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
            # if msg.servo >=  0.25101073177496236 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6523243920226947 ) /  -0.0072966120045042234
            # elif msg.servo <=  0.25101073177496636  and msg.servo >=  0.2314058363891589 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6104338138481035 ) /  -0.006534965128602493
            # elif msg.servo <=  0.23140583638915363  and msg.servo >=  0.20987486744762235 :
            #     self.servo_pwm = (float(msg.servo) +  -0.543604886041357 ) /  -0.005382742235382818
            # elif msg.servo <=  0.20987486744762474  and msg.servo >=  0.18040649551999455 :
            #     self.servo_pwm = (float(msg.servo) +  -0.8188878872853146 ) /  -0.009822790642543384
            # elif msg.servo <=  0.18040649551999022  and msg.servo >=  0.15755732136543577 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6754719355353359 ) /  -0.007616391384851472
            # elif msg.servo <=  0.15755732136544343  and msg.servo >=  0.13513720545829078 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6657466152609044 ) /  -0.007473371969050896
            # elif msg.servo <=  0.13513720545828767  and msg.servo >=  0.10725378496736959 :
            #     self.servo_pwm = (float(msg.servo) +  -0.7950448237433487 ) /  -0.009294473496972691
            # elif msg.servo <=  0.10725378496736937  and msg.servo >=  0.08572935110935054 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6381898201318317 ) /  -0.007174811286006248
            # elif msg.servo <=  0.08572935110934932  and msg.servo >=  0.06382835221027816 :
            #     self.servo_pwm = (float(msg.servo) +  -0.647854989518842 ) /  -0.007300332966357048
            # elif msg.servo <=  0.06382835221027916  and msg.servo >=  0.03931265941845419 :
            #     self.servo_pwm = (float(msg.servo) +  -0.7175801599922763 ) /  -0.008171897597274965
            # elif msg.servo <=  0.03931265941845341  and msg.servo >=  0.017708566164701067 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6370259061056011 ) /  -0.007201364417917441
            # elif msg.servo <=  0.017708566164702844  and msg.servo >=  0.006036701440005454 :
            #     self.servo_pwm = (float(msg.servo) +  -0.519598749326691 ) /  -0.0058359323623487
            # elif msg.servo <=  0.006036701440003789  and msg.servo >=  0.00015487366844557648 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5236375453371245 ) /  -0.00588182777155819
            # elif msg.servo <=  0.00015487366844546546  and msg.servo >=  -0.010405592614529113 :
            #     self.servo_pwm = (float(msg.servo) +  -0.9400363728531845 ) /  -0.010560466282974596
            # elif msg.servo <=  -0.010405592614527559  and msg.servo >=  -0.023525084085319947 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5799715235711307 ) /  -0.006559745735396203
            # elif msg.servo <=  -0.023525084085320835  and msg.servo >=  -0.04090645768596346 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5095037063343894 ) /  -0.0057937912002142405
            # elif msg.servo <=  -0.04090645768596424  and msg.servo >=  -0.059341934182953526 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5428836313853643 ) /  -0.006145158832329774
            # elif msg.servo <=  -0.05934193418295319  and msg.servo >=  -0.07892108877267956 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5802437824147734 ) /  -0.006526384863242108
            # elif msg.servo <=  -0.07892108877267945  and msg.servo >=  -0.10429297192570863 :
            #     self.servo_pwm = (float(msg.servo) +  -0.775265644045968 ) /  -0.008457294384343044
            # elif msg.servo <=  -0.10429297192570874  and msg.servo >=  -0.1273614764160773 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6954151837403996 ) /  -0.007689501496789503
            # elif msg.servo <=  -0.1273614764160773  and msg.servo >=  -0.14760746289270177 :
            #     self.servo_pwm = (float(msg.servo) +  -0.594745374583529 ) /  -0.0067486621588748255
            # elif msg.servo <=  -0.147607462892704  and msg.servo >=  -0.16592955942563692 :
            #     self.servo_pwm = (float(msg.servo) +  -0.8601078464186098 ) /  -0.00916104826646649
            # elif msg.servo <=  -0.16592955942563659  and msg.servo >=  -0.1857666259072659 :
            #     self.servo_pwm = (float(msg.servo) +  -0.5746542558885244 ) /  -0.0066123554938764375
            # elif msg.servo <=  -0.185766625907263  and msg.servo >=  -0.20401013544697888 :
            #     self.servo_pwm = (float(msg.servo) +  -0.513567906448514 ) /  -0.006081169846571974
            # elif msg.servo <=  -0.2040101354469821  and msg.servo >=  -0.22493867244578258 :
            #     self.servo_pwm = (float(msg.servo) +  -0.6191789865058376 ) /  -0.006976178999600167
            # elif msg.servo <=  -0.22493867244577548  and msg.servo >=  -0.24201984478136818 :
            #     self.servo_pwm = (float(msg.servo) +  -0.464001945089797 ) /  -0.005693724111864235
            # elif msg.servo <=  -0.24201984478137384  and msg.servo >=  -0.2589552247315977 :
            #     self.servo_pwm = (float(msg.servo) +  -0.4579758598278809 ) /  -0.005645126650074635
            # elif msg.servo <=  -0.2589552247315916 :
            #     self.servo_pwm = (float(msg.servo) +  -1.017130952562731 ) /  -0.010047922655860807
            if msg.servo >=  0.260288022001283 :
                self.servo_pwm = (float(msg.servo) +  -0.5651922493796864 ) /  -0.005256969437558679
            elif msg.servo <=  0.26028802200128004  and msg.servo >=  0.23497453767000254 :
                self.servo_pwm = (float(msg.servo) +  -0.7496820524059791 ) /  -0.008437828110425845
            elif msg.servo <=  0.23497453767000032  and msg.servo >=  0.21265485250555205 :
                self.servo_pwm = (float(msg.servo) +  -0.688808136013782 ) /  -0.007439895054816093
            elif msg.servo <=  0.21265485250554994  and msg.servo >=  0.18909597205593798 :
                self.servo_pwm = (float(msg.servo) +  -0.7152443020972729 ) /  -0.00785296014987067
            elif msg.servo <=  0.18909597205594664  and msg.servo >=  0.16792026992896686 :
                self.servo_pwm = (float(msg.servo) +  -0.6620199862251609 ) /  -0.007058567375659915
            elif msg.servo <=  0.1679202699289596  and msg.servo >=  0.14157354462293603 :
                self.servo_pwm = (float(msg.servo) +  -0.7826771937361767 ) /  -0.00878224176867453
            elif msg.servo <=  0.14157354462294186  and msg.servo >=  0.12297276730184514 :
                self.servo_pwm = (float(msg.servo) +  -0.5941924594362951 ) /  -0.006200259107032236
            elif msg.servo <=  0.1229727673018457  and msg.servo >=  0.0933773975505996 :
                self.servo_pwm = (float(msg.servo) +  -0.8727221343334117 ) /  -0.009865123250415343
            elif msg.servo <=  0.09337739755059238  and msg.servo >=  0.06989588849752426 :
                self.servo_pwm = (float(msg.servo) +  -0.7117238026147196 ) /  -0.00782716968435604
            elif msg.servo <=  0.06989588849753003  and msg.servo >=  0.04456945820741365 :
                self.servo_pwm = (float(msg.servo) +  -0.7621516497607108 ) /  -0.00844214343003879
            elif msg.servo <=  0.044569458207411705  and msg.servo >=  0.02855604903973913 :
                self.servo_pwm = (float(msg.servo) +  -0.4982827179581349 ) /  -0.005337803055890861
            elif msg.servo <=  0.028556049039735965  and msg.servo >=  0.010485807366451882 :
                self.servo_pwm = (float(msg.servo) +  -0.823646682664232 ) /  -0.009035120836642001
            elif msg.servo <=  0.010485807366456656  and msg.servo >=  0.0034435249809816604 :
                self.servo_pwm = (float(msg.servo) +  -0.6442912220592043 ) /  -0.0070422823854749735
            elif msg.servo <=  0.003443524980980217  and msg.servo >=  0.0007927721751040118 :
                self.servo_pwm = (float(msg.servo) +  -0.24466203031571498 ) /  -0.002650752805876206
            elif msg.servo <=  0.0007927721751049832  and msg.servo >=  -0.004695318450840702 :
                self.servo_pwm = (float(msg.servo) +  -0.5056971097621071 ) /  -0.005488090625945676
            elif msg.servo <=  -0.004695318450839481  and msg.servo >=  -0.028492632017824815 :
                self.servo_pwm = (float(msg.servo) +  -0.7330214021257045 ) /  -0.007932437855661764
            elif msg.servo <=  -0.02849263201782909  and msg.servo >=  -0.044887207874022395 :
                self.servo_pwm = (float(msg.servo) +  -0.4961337953803571 ) /  -0.005464858618731107
            elif msg.servo <=  -0.0448872078740189  and msg.servo >=  -0.06471304469101713 :
                self.servo_pwm = (float(msg.servo) +  -0.6093654070869237 ) /  -0.006608612272332753
            elif msg.servo <=  -0.06471304469101968  and msg.servo >=  -0.09247722762618604 :
                self.servo_pwm = (float(msg.servo) +  -0.8792691751046395 ) /  -0.009254727645055482
            elif msg.servo <=  -0.09247722762618427  and msg.servo >=  -0.11337442155949018 :
                self.servo_pwm = (float(msg.servo) +  -0.6389245600395232 ) /  -0.006965731311101976
            elif msg.servo <=  -0.1133744215594884  and msg.servo >=  -0.1339995665792617 :
                self.servo_pwm = (float(msg.servo) +  -0.6291307991523477 ) /  -0.006875048339924409
            elif msg.servo <=  -0.1339995665792687  and msg.servo >=  -0.15915354253826108 :
                self.servo_pwm = (float(msg.servo) +  -0.7966975439034497 ) /  -0.008384658652997463
            elif msg.servo <=  -0.1591535425382563  and msg.servo >=  -0.17362643158458213 :
                self.servo_pwm = (float(msg.servo) +  -0.6658011331023193 ) /  -0.007236444523162943
            elif msg.servo <=  -0.17362643158458613  and msg.servo >=  -0.19745731968899116 :
                self.servo_pwm = (float(msg.servo) +  -0.7478345751190756 ) /  -0.007943629368135014
            elif msg.servo <=  -0.1974573196889851  and msg.servo >=  -0.2150231817576554 :
                self.servo_pwm = (float(msg.servo) +  -0.4993218757016024 ) /  -0.005855287356223425
            elif msg.servo <=  -0.21502318175765056  and msg.servo >=  -0.23720068349417212 :
                self.servo_pwm = (float(msg.servo) +  -0.6868618888608925 ) /  -0.007392500578840517
            elif msg.servo <=  -0.23720068349417678  and msg.servo >=  -0.25321503212534924 :
                self.servo_pwm = (float(msg.servo) +  -0.4300638428046736 ) /  -0.005338116210390803
            elif msg.servo <=  -0.2532150321253518 :
                self.servo_pwm = (float(msg.servo) +  -0.9014698174165597 ) /  -0.009020975387046184





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
