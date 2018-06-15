#!/usr/bin/env python
"""
    File name: visualizeCar.py
    Author: Shuqi Xu
    Email: shuqixu@kth.se
    Python Version: 2.7.12
"""
# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

import rospy
import os
import sys
homedir = os.path.expanduser("~")
sys.path.append(os.path.join(homedir,"barc/workspace/src/barc/src/library"))
from Localization_helpers import Track
from barc.msg import ECU, pos_info, Vel_est, mpc_visual
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_imu_fusion, hedge_pos
from numpy import eye, array, zeros, cos, sin, vstack, append
from numpy import ones, size, matrix
import matplotlib.pyplot as plt
import numpy as np


class Plotter(object):
    """ 
    doc string
    """
    def __init__(self):
        # NODE INITIALIZATION
        self.s_prev = 0.0

        # FIGURE INITIALIZATION
        plt.ion()
        self.fig = plt.figure("Race",figsize=(12,8))
        self.ax  = self.fig.add_subplot(1,1,1)

        # PLOT DATA INITIALIZATION
        self.x = 0.0
        self.y = 0.0
        self.state = zeros(8) # infomation about vehicle current state
        car_dx = 2*rospy.get_param("L_a")
        car_dy = 0.125
        car_x = [car_dx, car_dx, -car_dx, -car_dx, car_dx]
        car_y = [car_dy, -car_dy, -car_dy, car_dy, car_dy]
        self.car_origin = vstack((array(car_x), array(car_y)))
        self.car_update = vstack((array(car_x), array(car_y)))
        self.car_h, = self.ax.plot(car_x,car_y,"k-")
        self.traj_x = []
        self.traj_y = []
        self.traj_h, = self.ax.plot(self.traj_x,self.traj_y,"b--")
        self.gps_x = []
        self.gps_y = []
        self.gps_h, = self.ax.plot(self.gps_x,self.gps_y,"g--")
        self.hedge_x = []
        self.hedge_y = []
        self.hedge_h, = self.ax.plot(self.hedge_x,self.hedge_y,"r")
        self.pre_x = zeros(rospy.get_param("controller/N"))
        self.pre_y = zeros(rospy.get_param("controller/N"))
        self.pre_h, = self.ax.plot(self.pre_x,self.pre_y,"b.-")
        self.SS_x = zeros(rospy.get_param("controller/Nl")*rospy.get_param("controller/Np"))
        self.SS_y = zeros(rospy.get_param("controller/Nl")*rospy.get_param("controller/Np"))
        self.SS_h, = self.ax.plot(self.SS_x,self.SS_y,"ro",alpha=0.3)
        self.sysID_x = zeros(rospy.get_param("controller/feature_Nl")*rospy.get_param("controller/feature_Np"))
        self.sysID_y = zeros(rospy.get_param("controller/feature_Nl")*rospy.get_param("controller/feature_Np"))
        self.sysID_h, = self.ax.plot(self.sysID_x,self.sysID_y,"go",alpha=0.3)

        # ATTRIBUTE FOR LAP TIME PLOT
        self.lapTime = zeros(5)  
        self.lapNum = [1]*5  
        self.cost = [0]*5
        self.v_avg = zeros(5)
        self.lapTime_s = [self.ax.annotate("Time: 0.00s, It: 0, v: 0.00 m/s", xy=[0.5,(0.6*i+5)/(len(self.lapTime)+10)], xycoords= "axes fraction", 
                                               fontsize=12, verticalalignment="top", horizontalalignment="center") for i in range(5)]

        rospy.init_node("visualizeCar")
        rospy.Subscriber("pos_info",         pos_info,         self.posInfo_callback, queue_size=1)
        rospy.Subscriber("hedge_imu_fusion", hedge_imu_fusion, self.gps_callback,     queue_size=1)
        rospy.Subscriber("hedge_pos",        hedge_pos,        self.hedge_callback,   queue_size=1)
        rospy.Subscriber("mpc_visual",       mpc_visual,       self.mpcVis_callback,  queue_size=1)

    def updatePlot(self):
        self.car_h.set_data(self.car_update[0],self.car_update[1])

        # estimator trajectory
        num = min(len(self.traj_x),len(self.traj_y))
        self.traj_h.set_data(self.traj_x[:num],self.traj_y[:num])
        # hedge_imu_fusion trajectory (100hz)
        num = min(len(self.gps_x),len(self.gps_y))
        self.gps_h.set_data(self.gps_x[:num],self.gps_y[:num])
        # hedge_pos trajectory (16hz)
        num = min(len(self.hedge_x),len(self.hedge_y))
        self.hedge_h.set_data(self.hedge_x[:num],self.hedge_y[:num])

        self.pre_h.set_data(self.pre_x,self.pre_y)
        self.SS_h.set_data(self.SS_x,self.SS_y)
        self.sysID_h.set_data(self.sysID_x,self.sysID_y)

        self.ax.set_xlabel("Lap: {}, vx: {} m/s, vy: {} m/s, ax: {} m/s2, ay: {} m/s2, psiDot: {} rad/s, a: {} m/s2, df: {} rad".format(int(self.state[7]), self.state[0], self.state[1], self.state[2], self.state[3], self.state[4], self.state[5], self.state[6]))

    def updateLapTime(self):
        if len(self.lapTime)<len(self.lapTime_s):
            for i in range(len(self.lapTime)):
                self.lapTime_s[i].set_text("Lap: {}, Time: {}s, It: {}, v: {} m/s".format(self.lapNum[-i-1], self.lapTime[-i-1],self.cost[-i-1],self.v_avg[-i-1]))
        else:
            for i in range(len(self.lapTime_s)):
                self.lapTime_s[i].set_text("Lap: {}, Time: {}s, It: {}, v: {} m/s".format(self.lapNum[-i-1], self.lapTime[-i-1],self.cost[-i-1],self.v_avg[-i-1]))

    def plotTrack(self,track):
        self.ax.plot(track.nodes[0,:],       track.nodes[1,:],       "k--", alpha=0.4)
        self.ax.plot(track.nodes_bound1[0,:],track.nodes_bound1[1,:],"r-")
        self.ax.plot(track.nodes_bound2[0,:],track.nodes_bound2[1,:],"r-")
        self.ax.grid('on')
        self.ax.axis('equal')
        plt.show()

    def posInfo_callback(self, posInfo):
        self.x = posInfo.x
        self.y = posInfo.y
        if posInfo.s<self.s_prev:
            self.traj_x = []
            self.traj_y = []
        else:
            self.traj_x.append(posInfo.x)
            self.traj_y.append(posInfo.y)
        self.s_prev = posInfo.s
        R = matrix([[cos(posInfo.psi), -sin(posInfo.psi)], [sin(posInfo.psi), cos(posInfo.psi)]])
        car = R * self.car_origin
        self.car_update[0] = car[0] + self.x
        self.car_update[1] = car[1] + self.y

    def gps_callback(self, gps):
        if self.s_prev<0.3:
            self.gps_x = []
            self.gps_y = []
        else:
            self.gps_x.append(gps.x_m)
            self.gps_y.append(gps.y_m)

    def hedge_callback(self, gps):
        if self.s_prev<0.3:
            self.hedge_x = []
            self.hedge_y = []
        else:
            self.hedge_x.append(gps.x_m)
            self.hedge_y.append(gps.y_m)

    def mpcVis_callback(self,mpc_vis):
        self.pre_x   = mpc_vis.z_x
        self.pre_y   = mpc_vis.z_y
        self.state   = np.round(mpc_vis.state,2)
        self.SS_x    = mpc_vis.SS_x
        self.SS_y    = mpc_vis.SS_y
        self.sysID_x = mpc_vis.z_iden_x
        self.sysID_y = mpc_vis.z_iden_y

        # LapTime related data update
        self.lapTime = mpc_vis.lapTime
        self.lapNum  = mpc_vis.lapNum
        self.cost    = mpc_vis.cost
        self.v_avg   = mpc_vis.v_avg

def main():
    plotter = Plotter()
    loop_rate = 50.0
    rate = rospy.Rate(loop_rate)
    
    track = Track(rospy.get_param("ds"),rospy.get_param("ey"))
    if rospy.get_param("feature_flag"):
        track.createFeatureTrack()
    else:
        track.createRaceTrack(rospy.get_param("race_track"))

    plotter.plotTrack(track)
    
    while not rospy.is_shutdown():
        plotter.updatePlot()
        plotter.updateLapTime()
        plotter.fig.canvas.draw()
        rate.sleep()
    
    # SAVING THE LAST FRAME OF FIGURE
    homedir = os.path.expanduser("~")
    pathSave = os.path.join(homedir,"BARC_visual.png")
    plotter.fig.savefig(pathSave,dpi=300)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass