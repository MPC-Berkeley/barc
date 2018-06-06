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
from marvelmind_nav.msg import hedge_pos
import numpy as np
from track_fnc import Map
from barc.msg import pos_info, prediction, SafeSetGlob
import matplotlib.pyplot as plt    
import pdb
import matplotlib.patches as patches

def main():
    rospy.init_node("realTimePlotting")
    StateView = False

    data = EstimationAndMesuredData()

    map = Map()

    loop_rate = 100
    rate = rospy.Rate(loop_rate)

    if StateView == True:
        fig, linevx, linevy, linewz, lineepsi, lineey, line_tr, line_pred = _initializeFigure(map)
    else:
        fig, line_tr, line_pred, line_SS, line_cl, rec = _initializeFigure_xy(map)


    ClosedLoopTraj_x = []
    ClosedLoopTraj_y = []

    while not rospy.is_shutdown():
        estimatedStates = data.readEstimatedData()
        s, ey, epsi, insideMap = map.getLocalPosition(estimatedStates[4], estimatedStates[5], estimatedStates[3])

        x = estimatedStates[4]
        y = estimatedStates[5]

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


        if StateView == True:
            linevx.set_data(0.0, estimatedStates[0])
            linevy.set_data(0.0, estimatedStates[1])
            linewz.set_data(0.0, estimatedStates[2])
            lineepsi.set_data(0.0, epsi)
            lineey.set_data(0.0, ey)
        
        line_tr.set_data(estimatedStates[4], estimatedStates[5])
        
        rec.set_xy(np.array([car_x, car_y]).T)

        if insideMap == 1:
            fig.canvas.draw()

        rate.sleep()

class EstimationAndMesuredData():
    """Object collecting closed loop data points
    Attributes:
        updateInitialConditions: function which updates initial conditions and clear the memory
    """
    def __init__(self):
        """Initialization
        Arguments:
            
        """
        rospy.Subscriber("hedge_pos", hedge_pos, self.gps_callback)
        rospy.Subscriber("pos_info", pos_info, self.pos_info_callback)
        rospy.Subscriber("OL_predictions", prediction, self.prediction_callback)
        rospy.Subscriber('SS', SafeSetGlob, self.SS_callback)

        self.s    = []
        self.ey   = []
        self.epsi = []

        self.SSx  = []
        self.SSy  = []

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
def _initializeFigure_xy(map):
    xdata = []; ydata = []
    fig = plt.figure()
    plt.ion()
    axtr = plt.axes()

    Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
    Points1 = np.zeros((Points, 2))
    Points2 = np.zeros((Points, 2))
    Points0 = np.zeros((Points, 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.width)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.width)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    line_cl, = axtr.plot(xdata, ydata, '-k')
    line_tr, = axtr.plot(xdata, ydata, '-or')
    line_SS, = axtr.plot(xdata, ydata, 'og')
    line_pred, = axtr.plot(xdata, ydata, '-or')
    
    v = np.array([[ 1.,  1.],
                  [ 1., -1.],
                  [-1., -1.],
                  [-1.,  1.]])

    rec = patches.Polygon(v, alpha=0.7,closed=True, fc='r', ec='k',zorder=10)
    axtr.add_patch(rec)

    plt.show()

    return fig, line_tr, line_pred, line_SS, line_cl, rec


def _initializeFigure(map):
    xdata = []; ydata = []
    fig = plt.figure()
    plt.ion()

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
