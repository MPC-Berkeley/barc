#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Author: J. Noonan
# Email: jpnoonan@berkeley.edu
#
# This code provides a way to see the car's trajectory, orientation, and velocity profile in 
# real time with referenced to the track defined a priori.
#
# ---------------------------------------------------------------------------
import sys
sys.path.append(sys.path[0]+'/ControllersObject')
sys.path.append(sys.path[0]+'/Utilities')
from sensor_msgs.msg import Imu
import rospy
from marvelmind_nav.msg import hedge_pos, hedge_imu_fusion
import numpy as np
from trackInitialization import Map
from barc.msg import pos_info, prediction, SafeSetGlob
import matplotlib.pyplot as plt    
import pdb
import matplotlib.patches as patches

def main():
    rospy.init_node("imuPlotting")

    imu = ImuClass(t0)

    StateView = False

    data = EstimationAndMesuredData()

    map = Map()

    loop_rate = 100
    rate = rospy.Rate(loop_rate)

    if StateView == True:
        fig, linevx, linevy, linewz, lineepsi, lineey, line_tr, line_pred = _initializeFigure(map)
    else:
        fig, axtr, line_tr, line_pred, line_SS, line_cl, line_gps_cl, rec = _initializeFigure_xy(map)

    ClosedLoopTraj_gps_x = []
    ClosedLoopTraj_gps_y = []
    ClosedLoopTraj_x = []
    ClosedLoopTraj_y = []
    maxVx = 0.0

    flagHalfLap = False

    while not rospy.is_shutdown():
        estimatedStates = data.readEstimatedData()
        s, ey, epsi, insideMap = map.getLocalPosition(estimatedStates[4], estimatedStates[5], estimatedStates[3])

        if s > map.TrackLength / 2:
            flagHalfLap = True

        if (s < map.TrackLength / 4) and (flagHalfLap == True): # New lap
            ClosedLoopTraj_gps_x = []
            ClosedLoopTraj_gps_y = []
            ClosedLoopTraj_x = []
            ClosedLoopTraj_y = []
            flagHalfLap = False

        x = estimatedStates[4]
        y = estimatedStates[5]

        ClosedLoopTraj_gps_x.append(data.MeasuredData[0]) 
        ClosedLoopTraj_gps_y.append(data.MeasuredData[1])

        ClosedLoopTraj_x.append(x) 
        ClosedLoopTraj_y.append(y)

        psi = estimatedStates[3]
        l = 0.4; w = 0.2
        car_x = [ x + l * np.cos(psi) - w * np.sin(psi), x + l*np.cos(psi) + w * np.sin(psi),
                  x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
        car_y = [ y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
                  y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]

        if data.s != []:
            xPredicted = np.zeros((len(data.s), 1))
            yPredicted = np.zeros((len(data.s), 1))
            for j in range(0, len(data.s)):
                sPred    = data.s[j]
                eyPred   = data.ey[j]
                epsiPred = data.epsi[j]

                xPredicted[j], yPredicted[j] = map.getGlobalPosition(sPred, eyPred)
            # print "sPred is: ", sPred
            # print "eyPred is: ", eyPred
            
            line_pred.set_data(xPredicted, yPredicted)

        if data.SSx != []:            
            line_SS.set_data(data.SSx, data.SSy)

        line_cl.set_data(ClosedLoopTraj_x, ClosedLoopTraj_y)
        line_gps_cl.set_data(ClosedLoopTraj_gps_x, ClosedLoopTraj_gps_y)


        if StateView == True:
            linevx.set_data(0.0, estimatedStates[0])
            linevy.set_data(0.0, estimatedStates[1])
            linewz.set_data(0.0, estimatedStates[2])
            lineepsi.set_data(0.0, epsi)
            lineey.set_data(0.0, ey)
        
        line_tr.set_data(estimatedStates[4], estimatedStates[5])
        
        rec.set_xy(np.array([car_x, car_y]).T)

        maxVx = np.maximum(maxVx, estimatedStates[0])

        StringValue = "vx: "+str(estimatedStates[0])+" max vx: "+str(maxVx)
        axtr.set_title(StringValue)
        
        if insideMap == 1:
            fig.canvas.draw()

        rate.sleep()

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
        (roll_raw, pitch_raw, dummy) = transformations.euler_from_quaternion(quaternion)
        self.roll   = roll_raw
        self.pitch  = pitch_raw

        w_z = data.angular_velocity.z
        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        a_z = data.linear_acceleration.z

        self.psiDot = w_z
        # Transformation from imu frame to vehicle frame (negative roll/pitch and reversed matrix multiplication to go back)
        self.ax = cos(-pitch_raw)*a_x + sin(-pitch_raw)*sin(-roll_raw)*a_y - sin(-pitch_raw)*cos(-roll_raw)*a_z
        self.ay = cos(-roll_raw)*a_y + sin(-roll_raw)*a_z

        self.prev_time = self.curr_time

    def saveHistory(self):
        """ Save measurement data into history array"""

        self.time_his.append(self.curr_time)
        
        self.yaw_his.append(self.yaw)
        self.psiDot_his.append(self.psiDot)
        self.ax_his.append(self.ax)
        self.ay_his.append(self.ay)
        self.roll_his.append(self.roll)
        self.pitch_his.append(self.pitch)


    
# ===================================================================================================================================== #
# ============================================================= Internal Functions ==================================================== #
# ===================================================================================================================================== #

def _initializeFigure(map):
    xdata = []; ydata = []
    fig = plt.figure()
    plt.ion()

    ax1 = fig.add_subplot(2, 1, 1)
    yawRate, = ax1.plot(xdata, ydata, 'or-')
    axvx.set_ylim([0, 1.5])
    plt.ylabel("yaw rate")
    plt.xlabel("t")

    ax2 = fig.add_subplot(2, 1, 2)
    yaw, = ax2.plot(xdata, ydata, 'or-')
    plt.ylabel("yaw")
    plt.xlabel("s")
    
    plt.show()

    return fig, yawRate, yaw

# ===================================================================================================================================== #
# ========================================================= End of Internal Functions ================================================= #
# ===================================================================================================================================== #

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
