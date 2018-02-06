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
import matplotlib.pyplot as plt
import pylab

x_gps   = None
y_gps   = None

def gps_callback(data):
    global x_gps, y_gps
    x_gps   = data.x_m
    y_gps   = data.y_m

def show():
    plt.show()

def view_trajectory():
    global x_gps, y_gps

    rospy.init_node("car_view_trajectory_node", anonymous=True)
    rospy.Subscriber("hedge_pos", hedge_pos, gps_callback)
    rospy.on_shutdown(show)
    
    fig = pylab.figure()
    ax1 = fig.add_subplot(1, 1, 1)
    ax1.axis('equal')
    ax1.grid('on')
    pylab.ion()
    xlim = 5
    ylim = 5

    loop_rate = 50
    rate = rospy.Rate(loop_rate)

    while not rospy.is_shutdown():
        if (x_gps and y_gps):
            ax1.plot(x_gps, y_gps, 'bo', label="GPS data path")
            ax1.set_xlim([-xlim, xlim])
            ax1.set_ylim([-ylim, ylim])

        ax1.set_title("Data from indoor GPS")
        pylab.pause(0.001)
        #pylab.gcf().clear()

if __name__ == '__main__':
    try:
        view_trajectory()
    except rospy.ROSInterruptException:
        pass
