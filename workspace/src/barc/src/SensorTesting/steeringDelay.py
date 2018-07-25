#!/usr/bin/env python

import thread
import sys
from sensor_msgs.msg import Imu
import rospy
from marvelmind_nav.msg import hedge_pos, hedge_imu_fusion
import numpy as np
from barc.msg import pos_info, prediction, SafeSetGlob, Vel_est
from tf import transformations
import matplotlib.pyplot as plt    
import pdb
import matplotlib.patches as patches
import numpy as np
from numpy import sin, cos
import pickle
from barc.msg import pos_info, ECU, prediction, SafeSetGlob


def input_thread(a_list):
    input = raw_input()
    a_list.append(input)

def main():
    rospy.init_node("steeringMap")

    input_commands = rospy.Publisher('ecu', ECU, queue_size = 1)
    t0 = rospy.get_rostime().to_sec()

    imu = ImuClass(t0)
    enc = EncClass(t0)
    cmd = ECU()

    fbk_srv = fbServoClass()

    loop_rate = 100
    rate = rospy.Rate(loop_rate)

    steering_his = []
    PWMsteering_his = []
    vx_his = []
    fbk_srv_his = []


    while True!=False:
        okFlag = False
        vx_his = []

        while okFlag==False:
            print "Enter rad for Steering 1"
            rad_Steering_1 = raw_input()
            print "Enter Acceleration 1"
            acc_Input_1 = raw_input()
            print "Enter rad for Steering 2"
            rad_Steering_2 = raw_input()
            print "Enter Acceleration 2"
            acc_Input_2 = raw_input()

            print "Steering you selected", PWM_Steering, ". Acceleration you selected", PWM_Input, ". Now enter ok to continue the test or q to exit"
            confirmInput = raw_input()
            okFlag = confirmInput == "ok"
            if confirmInput == "q":
                # Save Data
                file_data = open(sys.path[0]+'/../data/sensorTesting/steeringMap_BARC'+'.obj', 'wb')
                pickle.dump(steering_his, file_data)
                pickle.dump(steering_his, file_data)
                pickle.dump(fbk_srv_his, file_data)
                file_data.close()
    
        a_list = [0]
        thread.start_new_thread(input_thread, (a_list,))
        timeCounter = 0
        while a_list[-1] != "c":
            cmd.servo = float(PWM_Steering_1)
            cmd.motor = float(PWM_Input_1)
            input_commands.publish(cmd)
            vx_his.append(enc.v_meas)

            if enc.v_meas>1.1:
                steering_his.append(rad_Steering_1)
                fbk_srv_his.append(fbk_srv.value)

            rate.sleep()

        while a_list[-1] != "s":
            cmd.servo = float(PWM_Steering_2)
            cmd.motor = float(PWM_Input_2)
            input_commands.publish(cmd)
            vx_his.append(enc.v_meas)

            if enc.v_meas>1.1:
                steering_his.append(rad_Steering_2)
                fbk_srv_his.append(fbk_srv.value)

            rate.sleep()


        cmd.servo = 90.0
        cmd.motor = 90.0
        input_commands.publish(cmd)

        if steering_his != []:
            # Plotting
            plt.figure(1)
            plt.plot(vx_his, '-o')
            plt.legend()
            plt.ylabel('vx_his')

            ax = plt.figure(2)
            plt.plot(steering_his, '-o', label="Commanded Steering")
            a = 0.1
            b = 0.1
            plt.plot([a*i+b for i in fbk_srv_his], '-s', label="Measured Steering")
            
            plt.legend()
            plt.ylabel('steering')
            plt.xlabel('PWM')
            plt.xlim((50, 150))

            # plt.xlim((50, 150))
            # pdb.set_trace()


            plt.show()    



# ================================================================================================================== #
# ================================================================================================================== #
# ================================================================================================================== #
# ================================================================================================================== #
def fbk_servo2angle(fbk_servo):
    value = 0.0
    if fbk_servo <=  262.855764711 :
        value = (float(fbk_servo) +  -325.701609958 ) /  200.146003972
    elif fbk_servo >=  262.77571644  and fbk_servo <=  276.316745277 :
        value = (float(fbk_servo) +  -323.71034621 ) /  194.059330479
    elif fbk_servo >=  277.146919121  and fbk_servo <=  286.84972625 :
        value = (float(fbk_servo) +  -322.426685723 ) /  185.403957881
    elif fbk_servo >=  285.978054091  and fbk_servo <=  298.381242153 :
        value = (float(fbk_servo) +  -324.959502285 ) /  203.14593732
    elif fbk_servo >=  298.982314807  and fbk_servo <=  307.363898115 :
        value = (float(fbk_servo) +  -330.413252214 ) /  240.23646426
    elif fbk_servo >=  307.42318524  and fbk_servo <=  321.275461559 :
        value = (float(fbk_servo) +  -329.191048027 ) /  226.879866922
    elif fbk_servo >=  321.181409436  and fbk_servo <=  325.220355282 :
        value = (float(fbk_servo) +  -329.259301128 ) /  231.531927481
    elif fbk_servo >=  325.514791039  and fbk_servo <=  329.236912923 :
        value = (float(fbk_servo) +  -329.236912923 ) /  213.370044281
    elif fbk_servo >=  329.748964883  and fbk_servo <=  344.203827941 :
        value = (float(fbk_servo) +  -329.748964883 ) /  236.749349448
    elif fbk_servo >=  344.1983397  and fbk_servo <=  350.412110906 :
        value = (float(fbk_servo) +  -329.699540221 ) /  237.46896326
    elif fbk_servo >=  350.445546932  and fbk_servo <=  356.912318647 :
        value = (float(fbk_servo) +  -328.889641217 ) /  247.137772538
    elif fbk_servo >=  355.26962867  and fbk_servo <=  368.204209132 :
        value = (float(fbk_servo) +  -321.63971947 ) /  296.589106128
    elif fbk_servo >=  367.549320682  and fbk_servo <=  377.272666065 :
        value = (float(fbk_servo) +  -323.794266458 ) /  278.694612891
    elif fbk_servo >=  377.798822582  and fbk_servo <=  386.633382013 :
        value = (float(fbk_servo) +  -329.208745715 ) /  253.219856284
    elif fbk_servo >=  386.632540987 :
        value = (float(fbk_servo) +  -328.733535618 ) /  255.311635631

    return value

class fbServoClass(object):
    def __init__(self):
        rospy.Subscriber('srv_fbk', ECU, self.srv_fbk_callback, queue_size=1)

        # ENC measurement
        self.recorded = [0.0]*int(10)
        self.value = 0.0

    def srv_fbk_callback(self,data):
        """Unpack message from sensor, ENC"""
        self.recorded.pop(0)
        self.recorded.append(data.servo)
        self.value = np.sum(self.recorded)/len(self.recorded)



class EncClass(object):
    """ Object collecting ENC measurement data
    Attributes:
        Measurement:
            1.v_fl 2.v_fr 3. v_rl 4. v_rr
        Measurement history:
            1.v_fl_his 2.v_fr_his 3. v_rl_his 4. v_rr_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        rospy.Subscriber('vel_est', Vel_est, self.enc_callback, queue_size=1)

        # ENC measurement
        self.v_fl      = 0.0
        self.v_fr      = 0.0
        self.v_rl      = 0.0
        self.v_rr      = 0.0
        self.v_meas    = 0.0
        
        # ENC measurement history
        self.v_fl_his    = []
        self.v_fr_his    = []
        self.v_rl_his    = []
        self.v_rr_his    = []
        self.v_meas_his  = []
        
        # time stamp
        self.v_count    = 0
        self.v_prev     = 0.0
        self.t0         = t0
        self.time_his   = []
        self.curr_time  = rospy.get_rostime().to_sec() - self.t0

    def enc_callback(self,data):
        """Unpack message from sensor, ENC"""
        self.curr_time = rospy.get_rostime().to_sec() - self.t0

        self.v_fl = data.vel_fl
        self.v_fr = data.vel_fr
        self.v_rl = data.vel_bl
        self.v_rr = data.vel_br
        v_est = (self.v_rl + self.v_rr)/2
        if v_est != self.v_prev:
            self.v_meas = v_est
            self.v_prev = v_est
            self.v_count = 0
        else:
            self.v_count += 1
            if self.v_count > 40:
                self.v_meas = 0       

        self.saveHistory()

    def saveHistory(self):
        self.time_his.append(self.curr_time)
        
        self.v_fl_his.append(self.v_fl)
        self.v_fr_his.append(self.v_fr)
        self.v_rl_his.append(self.v_rl)
        self.v_rr_his.append(self.v_rr)

        self.v_meas_his.append(self.v_meas)

class ImuClass(object):
    """ Object collecting GPS measurement data
    Attributes:
        Measurement:
            1.yaw 2.psiDot 3.ax 4.ay 5.roll 6.pitch
        Measurement history:
            1.yaw_his 2.psiDot_his 3.ax_his 4.ay_his 5.roll_his 6.pitch_his
        Time stamp
            1.t0 2.time_his
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """

        rospy.Subscriber('imu/data', Imu, self.imu_callback, queue_size=1)
        input_commands = rospy.Publisher('ecu', ECU, queue_size=1)

        # Imu measurement
        self.running = False
        self.yaw_prev= 0.0
        self.yaw     = 0.0
        self.yawInt  = 0.0
        self.psiDot  = 0.0
        self.ax      = 0.0
        self.ay      = 0.0
        self.az      = 0.0
        self.roll    = 0.0
        self.pitch   = 0.0
        
        # Imu measurement history
        self.yaw_his     = []
        self.psiDot_his  = []
        self.ax_his      = []
        self.ay_his      = []
        self.roll_his    = []
        self.pitch_his   = []
        
        # time stamp
        self.t0          = t0
        self.time_his    = []

        # Time for yawDot integration
        self.curr_time = rospy.get_rostime().to_sec()
        self.prev_time = self.curr_time

    def imu_callback(self,data):
        """Unpack message from sensor, IMU"""
        
        self.curr_time = rospy.get_rostime().to_sec() - self.t0

        if self.prev_time > 0:
            self.yawInt += self.psiDot * (self.curr_time-self.prev_time)
   
        ori = data.orientation
        
        quaternion = (ori.x, ori.y, ori.z, ori.w)
        (roll_raw, pitch_raw, yaw_raw) = transformations.euler_from_quaternion(quaternion)
        self.roll   = roll_raw
        self.pitch  = pitch_raw

        # yaw_meas is element of [-pi,pi]
        yaw = np.unwrap([self.yaw_prev, yaw_raw])[1]       # get smooth yaw (from beginning)
        self.yaw_prev = self.yaw                   # and always use raw measured yaw for unwrapping
        # from this point on 'yaw' will be definitely unwrapped (smooth)!
        if not self.running:
            self.yaw0 = yaw              # set yaw0 to current yaw
            self.yaw = 0                 # and current yaw to zero
            self.running = True
        else:
            self.yaw = yaw - self.yaw0


        w_z = data.angular_velocity.z
        w_x = data.angular_velocity.x
        w_y = data.angular_velocity.y

        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        a_z = data.linear_acceleration.z

        self.psiDot = w_y * sin(roll_raw) / cos(pitch_raw) + w_z * cos(roll_raw) / cos(pitch_raw)
        # Transformation from imu frame to vehicle frame (negative roll/pitch and reversed matrix multiplication to go back)
        # self.ax = cos(-pitch_raw)*a_x + sin(-pitch_raw)*sin(-roll_raw)*a_y - sin(-pitch_raw)*cos(-roll_raw)*a_z
        # self.ay = cos(-roll_raw)*a_y + sin(-roll_raw)*a_z
        self.ax = a_x
        self.ay = a_y
        self.az = a_z

        self.prev_time = self.curr_time


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass