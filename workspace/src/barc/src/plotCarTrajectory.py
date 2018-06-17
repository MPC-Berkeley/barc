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

import rospy
from marvelmind_nav.msg import hedge_pos, hedge_imu_fusion
import numpy as np
from trackInitialization import Map
from barc.msg import pos_info, prediction, SafeSetGlob, simulatorStates
import matplotlib.pyplot as plt    
import pdb
import matplotlib.patches as patches

def main():
    rospy.init_node("realTimePlotting")
    StateView = False

    mode = rospy.get_param("/control/mode")
    data = EstimationAndMesuredData(mode)        

    map = Map()

    loop_rate = 100
    rate = rospy.Rate(loop_rate)

    if StateView == True:
        fig, linevx, linevy, linewz, lineepsi, lineey, line_tr, line_pred = _initializeFigure(map)
    else:
        fig, axtr, line_tr, line_pred, line_SS, line_cl, line_gps_cl, rec, rec_sim = _initializeFigure_xy(map, mode)

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
        
        car_x, car_y = getCarPosition(x, y, psi, w, l)
        rec.set_xy(np.array([car_x, car_y]).T)
        
        if mode == "simulations":
            x_sim   = data.sim_x[-1]
            y_sim   = data.sim_y[-1]
            psi_sim = data.sim_psi[-1]
            car_sim_x, car_sim_y = getCarPosition(x_sim, y_sim, psi_sim, w, l)
            rec_sim.set_xy(np.array([car_sim_x, car_sim_y]).T)

        maxVx = np.maximum(maxVx, estimatedStates[0])
    
        vSwitch      = 1.0
        psiSwitch    = 0.5 * 2.0

        StringValue = "vx: "+str(estimatedStates[0])+" max vx: "+str(maxVx)+" psiDot: "+str(estimatedStates[2])+" No vy: "+str(estimatedStates[0] > vSwitch or np.abs(estimatedStates[3]) > psiSwitch)
        axtr.set_title(StringValue)
        
        if insideMap == 1:
            fig.canvas.draw()

        rate.sleep()

def getCarPosition(x, y, psi, w, l):
    car_x = [ x + l * np.cos(psi) - w * np.sin(psi), x + l*np.cos(psi) + w * np.sin(psi),
              x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
    car_y = [ y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
              y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]
    return car_x, car_y

class EstimationAndMesuredData():
    """Object collecting closed loop data points
    Attributes:
        updateInitialConditions: function which updates initial conditions and clear the memory
    """
    def __init__(self, mode):
        """Initialization
        Arguments:
            
        """
        if mode == "simulations":
            rospy.Subscriber("simulatorStates", simulatorStates, self.simState_callback)

        rospy.Subscriber("hedge_imu_fusion", hedge_imu_fusion, self.gps_callback)
        rospy.Subscriber("pos_info", pos_info, self.pos_info_callback)
        rospy.Subscriber("OL_predictions", prediction, self.prediction_callback)
        rospy.Subscriber('SS', SafeSetGlob, self.SS_callback)

        self.s    = []
        self.ey   = []
        self.epsi = []

        self.SSx  = []
        self.SSy  = []

        self.MeasuredData = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.EstimatedData= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.sim_x   = [0.0]
        self.sim_y   = [0.0]
        self.sim_psi = [0.0]

    def simState_callback(self, msg):
        self.sim_x.append(msg.x)
        self.sim_y.append(msg.y)
        self.sim_psi.append(msg.psi)

    def gps_callback(self, msg):
        self.MeasuredData = [msg.x_m, msg.y_m]

    def pos_info_callback(self, msg):
        self.EstimatedData = [msg.v_x, msg.v_y, msg.psiDot, msg.psi, msg.x, msg.y]

    def prediction_callback(self, msg):
        self.s    = msg.s
        self.ey   = msg.ey
        self.epsi = msg.epsi

    def SS_callback(self, msg):
        self.SSx  = msg.SSx
        self.SSy  = msg.SSy

    def readEstimatedData(self):
        return self.EstimatedData


    
# ===================================================================================================================================== #
# ============================================================= Internal Functions ==================================================== #
# ===================================================================================================================================== #
def _initializeFigure_xy(map, mode):
    xdata = []; ydata = []
    fig = plt.figure(figsize=(12,8))
    plt.ion()
    axtr = plt.axes()

    Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
    Points1 = np.zeros((Points, 2))
    Points2 = np.zeros((Points, 2))
    Points0 = np.zeros((Points, 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    line_cl, = axtr.plot(xdata, ydata, '-k')
    line_gps_cl, = axtr.plot(xdata, ydata, '--og')
    line_tr, = axtr.plot(xdata, ydata, '-or')
    line_SS, = axtr.plot(xdata, ydata, 'og')
    line_pred, = axtr.plot(xdata, ydata, '-or')
    
    v = np.array([[ 1.,  1.],
                  [ 1., -1.],
                  [-1., -1.],
                  [-1.,  1.]])

    rec = patches.Polygon(v, alpha=0.7,closed=True, fc='r', ec='k',zorder=10)
    axtr.add_patch(rec)

    rec_sim = patches.Polygon(v, alpha=0.7,closed=True, fc='G', ec='k',zorder=10)

    if mode == "simulations":
        axtr.add_patch(rec_sim)

    plt.show()

    return fig, axtr, line_tr, line_pred, line_SS, line_cl, line_gps_cl, rec, rec_sim


def _initializeFigure(map):
    xdata = []; ydata = []
    plt.ion()
    fig = plt.figure(figsize=(40,20))

    axvx = fig.add_subplot(3, 2, 1)
    linevx, = axvx.plot(xdata, ydata, 'or-')
    axvx.set_ylim([0, 1.5])
    plt.ylabel("vx")
    plt.xlabel("t")

    axvy = fig.add_subplot(3, 2, 2)
    linevy, = axvy.plot(xdata, ydata, 'or-')
    plt.ylabel("vy")
    plt.xlabel("s")

    axwz = fig.add_subplot(3, 2, 3)
    linewz, = axwz.plot(xdata, ydata, 'or-')
    plt.ylabel("wz")
    plt.xlabel("s")

    axepsi = fig.add_subplot(3, 2, 4)
    lineepsi, = axepsi.plot(xdata, ydata, 'or-')
    axepsi.set_ylim([-np.pi/2,np.pi/2])
    plt.ylabel("epsi")
    plt.xlabel("s")

    axey = fig.add_subplot(3, 2, 5)
    lineey, = axey.plot(xdata, ydata, 'or-')
    axey.set_ylim([-map.width,map.width])
    plt.ylabel("ey")
    plt.xlabel("s")

    Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
    Points1 = np.zeros((Points, 2))
    Points2 = np.zeros((Points, 2))
    Points0 = np.zeros((Points, 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.width)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.width)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    axtr = fig.add_subplot(3, 2, 6)
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    line_tr, = axtr.plot(xdata, ydata, '-or')
    line_pred, = axtr.plot(xdata, ydata, '-or')
    
    plt.show()

    return fig, linevx, linevy, linewz, lineepsi, lineey, line_tr, line_pred

# ===================================================================================================================================== #
# ========================================================= End of Internal Functions ================================================= #
# ===================================================================================================================================== #

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
