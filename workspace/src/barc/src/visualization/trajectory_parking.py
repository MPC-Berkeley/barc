#!/usr/bin/env python

import rospy
import os
from barc.msg import Z_KinBkMdl, mpcSol
from numpy import array, ones, sin, cos, tan, vstack, matrix, roll
import matplotlib.pyplot as plt
import numpy as np
from auxilary import SimData

def main():

    # instantiate node and subs / pubs
    simData = SimData()
    rospy.init_node("graph", anonymous=True)
    rospy.Subscriber("plant", Z_KinBkMdl, simData.stateCallback, queue_size=1)
    rospy.Subscriber("mpcSolution", mpcSol, simData.mpcCallback, queue_size=1)

    # define node rate
    loop_rate   = 20.0
    rate        = rospy.Rate(loop_rate)

    # get vehicle parameters
    L   = rospy.get_param("/vehicle_length")  
    c   = rospy.get_param("/vehicle_width") / 2.0
    simData.setVehicleParameters(L,c)

    # get / set initial state
    s0      = rospy.get_param("/initial_state")
    simData.setInitialState( s0[0:3])

    # load solution data
    filepath    = os.path.expanduser("~") + "/barc/workspace/src/barc/src/control/parking/data/npz/sp10.npz"
    sp10        = np.load(filepath)
    xRef        = sp10[0,:]
    yRef        = sp10[1,:]

    # instantiate figure
    fig             = plt.figure(figsize=(20,20))
    ax1             = fig.add_subplot(1, 1, 1)
    plt.ion()
    ax1.grid('on')
    ax1.axis('equal')

    # draw objects
    simData.updateVhCorners()
    car_edges_plot,     = ax1.plot(simData.xc, simData.yc,"k-")
    car_center_dot,     = ax1.plot(simData.x, simData.y, "ro")
    #path_plot,         = ax1.plot(simData.x_hist, simData.y_hist, 'g-') # label="position history")
    target_path_plot,   = ax1.plot(xRef, yRef,'k-',label='target path')
    prediction_plot,    = ax1.plot(simData.z1OL, simData.z2OL,'r*',label='prediction')

    # draw obstacles
    # for plotting
    lObPlot = [     [ [-20,5],  [-1.3,5],   [-1.3,-5],  [-20,-5], [-20,5] ]  ,
                    [ [1.3,5],  [20,5],     [20,-5],    [1.3,-5], [1.3,5] ] ,
                    [ [-20,15], [20,15],    [20,11],    [-20,11], [-20,15]] ] # vertices given in CLOCK-WISE direction
    nObPlot =  3        # number of obstacles
    vObPlot = [4,4,4]   # number of vertices of each obstacle, vector of dimenion nOb

    for j in range(nObPlot):
        for k in range(vObPlot[j]):
            ax1.plot([lObPlot[j][k][0],lObPlot[j][k+1][0]] , [lObPlot[j][k][1],lObPlot[j][k+1][1]] ,"k")

    while not rospy.is_shutdown():
        
        # update drawing
        simData.updateVhCorners()
        car_edges_plot.set_data(simData.xc, simData.yc)
        car_center_dot.set_data(simData.x, simData.y)
        #path_plot.set_data(simData.x_hist, simData.y_hist)
        prediction_plot.set_data(simData.z1OL, simData.z2OL)

        # refresh screen
        fig.canvas.draw()
        plt.show()
        rate.sleep()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
