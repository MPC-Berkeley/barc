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


def main():
    rospy.init_node("imuPlotting")

    carSel = rospy.get_param("/IMUplot/car")

    t0 = rospy.get_rostime().to_sec()
    imu = ImuClass(t0)

    loop_rate = 100
    rate = rospy.Rate(loop_rate)

    fig, ax1, ax2, yawRatePlot, yawMeaPlot, yawIntPlot = _initializeFigureIMU()

    yaw_raw = []
    yaw_int = []
    yawRate = []
    time    = []
    
    counter = 0
    while not rospy.is_shutdown():
        yaw_raw.append(imu.yaw) 
        yaw_int.append(imu.yawInt)
        yawRate.append(imu.psiDot)
        time.append(counter)
        counter += 1

        yawRatePlot.set_data(time, yawRate)
        ax1.set_xlim([0, counter])
        ax1.set_ylim([-10, 10])
        
        yawMeaPlot.set_data(time, yaw_raw)
        yawIntPlot.set_data(time, yaw_int)
        ax2.set_xlim([0, counter])
        ax2.set_ylim([-10, 10])
        
        fig.canvas.draw()

        rate.sleep()

    # Save Data
    file_data = open(sys.path[0]+'/../data/sensorTesting/IMU_'+carSel+'BARC'+'.obj', 'wb')
    pickle.dump(time, file_data)
    pickle.dump(yawRate, file_data)
    pickle.dump(yaw_raw, file_data)   
    pickle.dump(yaw_int, file_data)   

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

        # Imu measurement
        self.running = False
        self.yaw_prev= 0.0
        self.yaw     = 0.0
        self.yawInt  = 0.0
        self.psiDot  = 0.0
        self.ax      = 0.0
        self.ay      = 0.0
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
        
        self.curr_time = rospy.get_rostime().to_sec()

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
        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        a_z = data.linear_acceleration.z

        self.psiDot = w_z
        # Transformation from imu frame to vehicle frame (negative roll/pitch and reversed matrix multiplication to go back)
        self.ax = cos(-pitch_raw)*a_x + sin(-pitch_raw)*sin(-roll_raw)*a_y - sin(-pitch_raw)*cos(-roll_raw)*a_z
        self.ay = cos(-roll_raw)*a_y + sin(-roll_raw)*a_z

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

    ax1 = fig.add_subplot(2, 1, 1)
    yawRatePlot, = ax1.plot(xdata, ydata, 'or-')
    plt.ylabel("yaw rate")
    plt.xlabel("t")

    ax2 = fig.add_subplot(2, 1, 2)
    yawMeaPlot, = ax2.plot(xdata, ydata, 'or-')
    yawIntPlot, = ax2.plot(xdata, ydata, 'og--')
    plt.ylabel("yaw")
    plt.xlabel("t")

    plt.show()

    return fig, ax1, ax2, yawRatePlot, yawMeaPlot, yawIntPlot


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
