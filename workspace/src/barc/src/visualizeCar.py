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
sys.path.append(os.path.join(homedir,"barc/workspace/src/barc/src/Library"))
from Localization_helpers import Track
from barc.msg import ECU, pos_info, Vel_est, mpc_visual
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_imu_fusion
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
        rospy.init_node("visualizeCar")
        rospy.Subscriber("pos_info",   pos_info,   self.posInfo_callback, queue_size=1)
        rospy.Subscriber("mpc_visual", mpc_visual, self.mpcVis_callback, queue_size=1)

        # FIGURE INITIALIZATION
        plt.ion()
        self.fig = plt.figure("Race")
        self.ax  = self.fig.add_subplot(1,1,1)

        # PLOT DATA INITIALIZATION
        self.x = 0.0
        self.y = 0.0
        self.s_prev = 0.0
        car_dx = 2*rospy.get_param("L_a")
        car_dy = 0.125
        car_x = [car_dx, car_dx, -car_dx, -car_dx, car_dx]
        car_y = [car_dy, -car_dy, -car_dy, car_dy, car_dy]
        self.car_origin = vstack((array(car_x), array(car_y)))
        self.car_update = vstack((array(car_x), array(car_y)))
        self.car_h, = self.ax.plot(car_x,car_y,"k-")
        self.traj_x = []
        self.traj_y = []
        self.traj_h, = self.ax.plot(self.traj_x,self.traj_y,"b")
        self.pre_x = zeros(rospy.get_param("controller/N"))
        self.pre_y = zeros(rospy.get_param("controller/N"))
        self.pre_h, = self.ax.plot(self.pre_x,self.pre_y,"b*-")
        self.SS_x = zeros(rospy.get_param("controller/Nl")*rospy.get_param("controller/Np"))
        self.SS_y = zeros(rospy.get_param("controller/Nl")*rospy.get_param("controller/Np"))
        self.SS_h, = self.ax.plot(self.SS_x,self.SS_y,"ro",alpha=0.3)
        self.sysID_x = zeros(rospy.get_param("controller/feature_Nl")*rospy.get_param("controller/feature_Np"))
        self.sysID_y = zeros(rospy.get_param("controller/feature_Nl")*rospy.get_param("controller/feature_Np"))
        self.sysID_h, = self.ax.plot(self.sysID_x,self.sysID_y,"go",alpha=0.3)

    def updatePlot(self):
        self.car_h.set_data(self.car_update[0],self.car_update[1])

        num = min(len(self.traj_x),len(self.traj_y))
        self.traj_h.set_data(self.traj_x[:num],self.traj_y[:num])

        self.pre_h.set_data(self.pre_x,self.pre_y)
        self.SS_h.set_data(self.SS_x,self.SS_y)
        self.sysID_h.set_data(self.sysID_x,self.sysID_y)

    def plotTrack(self,track):
        self.ax.plot(track.nodes[0,:],       track.nodes[1,:],       "k--", alpha=0.4)
        self.ax.plot(track.nodes_bound1[0,:],track.nodes_bound1[1,:],"r-")
        self.ax.plot(track.nodes_bound2[0,:],track.nodes_bound2[1,:],"r-")
        # self.ax.grid('on')
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

    def mpcVis_callback(self,mpc_vis):
        self.pre_x   = mpc_vis.z_x
        self.pre_y   = mpc_vis.z_y
        self.SS_x    = mpc_vis.SS_x
        self.SS_y    = mpc_vis.SS_y
        self.sysID_x = mpc_vis.z_iden_x
        self.sysID_y = mpc_vis.z_iden_y

def main():
    plotter = Plotter()
    loop_rate = 50.0
    rate = rospy.Rate(loop_rate)
    
    track = Track(rospy.get_param("ds"),rospy.get_param("ey"))
    if rospy.get_param("feature_flag"):
        track.createFeatureTrack()
    else:
        track.createRaceTrack()

    plotter.plotTrack(track)
    
    while not rospy.is_shutdown():
        plotter.updatePlot()
        plotter.fig.canvas.draw()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass