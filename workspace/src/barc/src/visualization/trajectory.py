#!/usr/bin/env python

import rospy
from barc.msg import Z_KinBkMdl, mpcSol
from numpy import array, zeros, ones, sin, cos, tan, vstack, matrix, roll
import matplotlib.pyplot as plt
import numpy as np
from auxilary import SimData

def main():

    # instantiate node and subs / pubs
    simData = SimData()
    rospy.init_node("graph", anonymous=True)
    rospy.Subscriber("plant", Z_KinBkMdl, simData.stateCallback, queue_size=1)

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

    # instantiate figure
    fig             = plt.figure(figsize=(20,20))
    ax1             = fig.add_subplot(1, 1, 1)
    plt.ion()
    ax1.grid('on')
    ax1.axis('equal')

    # draw objects
    simData.updateVhCorners()
    car_plot,       = ax1.plot(simData.xc, simData.yc,"k-")
    boundary_plot,  = ax1.plot(10*simData.xVhCorners,10*simData.yVhCorners,"k-",alpha=0.4)
    center_plot,    = ax1.plot(simData.x, simData.y, "ro")
    path_plot,      = ax1.plot(simData.x_hist, simData.y_hist, 'g-') # label="position history")

    while not rospy.is_shutdown():
        

        # update drawing
        simData.updateVhCorners()
        car_plot.set_data(simData.xc, simData.yc)
        center_plot.set_data(simData.x, simData.y)
        path_plot.set_data(simData.x_hist, simData.y_hist)
        boundary_plot.set_data(10*simData.xVhCorners,10*simData.yVhCorners)

        # refresh screen
        fig.canvas.draw()
        plt.show()
        rate.sleep()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
