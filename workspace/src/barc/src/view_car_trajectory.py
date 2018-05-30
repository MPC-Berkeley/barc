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

import rospy
from marvelmind_nav.msg import hedge_pos
import numpy as np
from track_fnc import Map
from barc.msg import pos_info
import matplotlib.pyplot as plt    

def main():
    rospy.init_node("realTimePlotting")

    data = EstimationAndMesuredData()

    map = Map()

    loop_rate = 100
    rate = rospy.Rate(loop_rate)

    fig, linevx, linevy, linewz, lineepsi, lineey, line_tr = _initializeFigure(map)
    
    while not rospy.is_shutdown():
        estimatedStates = data.readEstimatedData()
        s, ey, epsi, insideMap = map.getLocalPosition(estimatedStates[4], estimatedStates[5], estimatedStates[3])

        linevx.set_data(0.0, estimatedStates[0])
        linevy.set_data(0.0, estimatedStates[1])
        linewz.set_data(0.0, estimatedStates[2])
        lineepsi.set_data(0.0, epsi)
        lineey.set_data(0.0, ey)
        line_tr.set_data(estimatedStates[4], estimatedStates[5])
        
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

    def gps_callback(self, msg):
        self.MeasuredData = [msg.x_m, msg.y_m]

    def pos_info_callback(self, msg):
        self.EstimatedData = [msg.v_x, msg.v_y, msg.psiDot, msg.psi, msg.x, msg.y]

    def readEstimatedData(self):
        return self.EstimatedData
    
# ===================================================================================================================================== #
# ============================================================= Internal Functions ==================================================== #
# ===================================================================================================================================== #
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
    plt.show()

    return fig, linevx, linevy, linewz, lineepsi, lineey, line_tr

# ===================================================================================================================================== #
# ========================================================= End of Internal Functions ================================================= #
# ===================================================================================================================================== #

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
