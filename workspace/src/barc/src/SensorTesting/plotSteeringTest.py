#!/usr/bin/env python

import sys
from sensor_msgs.msg import Imu
import rospy
from marvelmind_nav.msg import hedge_pos, hedge_imu_fusion
import numpy as np
from barc.msg import pos_info, prediction, SafeSetGlob
from tf import transformations
import matplotlib.pyplot as plt    
import pdb
import matplotlib.patches as patches
import numpy as np
from numpy import sin, cos
import pickle
from barc.msg import pos_info, ECU, prediction, SafeSetGlob

def main():
    rospy.init_node("imuPlotting")

    carSel = rospy.get_param("/IMUplot/car")
    input_commands = rospy.Publisher('ecu', ECU, queue_size=1)

    t0 = rospy.get_rostime().to_sec()
    imu = ImuClass(t0)

    loop_rate = 100
    rate = rospy.Rate(loop_rate)
    cmd = ECU()

    fig, ax1, ax2, ax3, ax4, yawRatePlot, yawMeaPlot, yawIntPlot, rollPlot, pitchPlot, axPlot, ayPlot, azPlot = _initializeFigureIMU()

    ax_raw    = []
    ay_raw    = []
    az_raw    = []
    yaw_raw   = []
    yaw_int   = []
    yawRate   = []
    roll_raw  = []
    pitch_raw = []
    steer_raw = []
    time      = []
    
    counter = 0


    while not rospy.is_shutdown():


        # ori = imu.orientation
        # quaternion = (ori.x, ori.y, ori.z, ori.w)
        # (roll_raw, pitch_raw, dummy) = transformations.euler_from_quaternion(quaternion)
        # self.roll   = roll_raw
        # self.pitch  = pitch_raw

        # w_z = imu.angular_velocity.z
        a_x = imu.ax
        a_y = imu.ay
        a_z = imu.az
        # a_z = imu.linear_acceleration.z
        
        if counter > 100:
            print "input_commands"
            cmd.servo = -0.0
            steer_raw.append(cmd.servo)
            cmd.motor = -0.1
            input_commands.publish(cmd)
        else:
            steer_raw.append(0.0)



        yaw_raw.append(imu.yaw) 
        yaw_int.append(imu.yawInt)
        yawRate.append(imu.psiDot)
        roll_raw.append(imu.roll)
        pitch_raw.append(imu.pitch)
        
        ax_raw.append(a_x)
        ay_raw.append(a_y)
        az_raw.append(a_z)



        time.append(counter)
        counter += 1

        yawRatePlot.set_data(time, yawRate)
        ax1.set_xlim([0, counter])
        ax1.set_ylim([-10, 10])
        
        yawMeaPlot.set_data(time, yaw_raw)
        yawIntPlot.set_data(time, yaw_int)
        ax2.set_xlim([0, counter])
        ax2.set_ylim([-10, 10])

        rollPlot.set_data(time, roll_raw)
        pitchPlot.set_data(time, pitch_raw)
        ax3.set_xlim([0, counter])
        ax3.set_ylim([-10, 10])

        axPlot.set_data(time, ax_raw)
        ayPlot.set_data(time, ay_raw)
        azPlot.set_data(time, az_raw)
        ax4.set_xlim([0, counter])
        ax4.set_ylim([-10, 10])
        
        fig.canvas.draw()

        rate.sleep()

    # Save Data
    file_data = open(sys.path[0]+'/../data/sensorTesting/Steering_'+carSel+'BARC'+'.obj', 'wb')
    pickle.dump(time, file_data)
    pickle.dump(yawRate, file_data)
    pickle.dump(yaw_raw, file_data)   
    pickle.dump(yaw_int, file_data)   
    pickle.dump(roll_raw, file_data)   
    pickle.dump(pitch_raw, file_data)      
    pickle.dump(ax_raw, file_data)   
    pickle.dump(ay_raw, file_data)
    pickle.dump(az_raw, file_data)
    pickle.dump(steer_raw, file_data)

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



    
# ===================================================================================================================================== #
# ============================================================= Internal Functions ==================================================== #
# ===================================================================================================================================== #

def _initializeFigureIMU():
    # xdata = []; ydata = []
    # fig = plt.figure()
    # plt.ion()

    xdata = []; ydata = []
    fig = plt.figure()
    plt.ion()

    ax1 = fig.add_subplot(4, 1, 1)
    yawRatePlot, = ax1.plot(xdata, ydata, 'or-')
    plt.ylabel("yaw rate")
    plt.xlabel("t")

    ax2 = fig.add_subplot(4, 1, 2)
    yawMeaPlot, = ax2.plot(xdata, ydata, 'or-')
    yawIntPlot, = ax2.plot(xdata, ydata, 'og--')
    plt.ylabel("yaw")
    plt.xlabel("t")

    ax3 = fig.add_subplot(4, 1, 3)
    rollPlot, = ax3.plot(xdata, ydata, 'or-', label="roll")
    pitchPlot, = ax3.plot(xdata, ydata, 'og--', label="pitch")
    plt.legend()
    plt.ylabel("rall/pitch")
    plt.xlabel("t")

    ax4 = fig.add_subplot(4, 1, 4)
    axPlot, = ax4.plot(xdata, ydata, 'or-', label="ax")
    ayPlot, = ax4.plot(xdata, ydata, 'og--', label="ay")
    azPlot, = ax4.plot(xdata, ydata, 'ok--', label="az")
    plt.legend()
    plt.xlabel("t")


    plt.show()

    return fig, ax1, ax2, ax3, ax4, yawRatePlot, yawMeaPlot, yawIntPlot, rollPlot, pitchPlot, axPlot, ayPlot, azPlot


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
