#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Author: J. Gonzales 
# Email: jon.gonzales@berkeley.edu
#
# Note: Code adapted from ideas in code from J. Noonan (jpnoonan@berkeley.edu)
#
# This code provides a way to see the car's trajectory, orientation, and velocity profile in 
# real time with referenced to the track defined a priori.
#
# ---------------------------------------------------------------------------

import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix
from lla2flat import lla2flat

class gps_viewer():
    def __init__(self):
        # define variables
        self.lat0   = None
        self.lng0   = None
        self.xt     = None
        self.yt     = None

        # launch figure
        self.ax1 = plt.subplot()
        self.ax1.grid('on')
        self.ax1.axis('equal')
        self.ax1.set_title("Data from GPS")
        self.xlim = 50.0
        self.ylim = 50.0
        plt.ion()

        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        rospy.on_shutdown(self.show)
        loop_rate = 50
        rate = rospy.Rate(loop_rate)

        while not rospy.is_shutdown():
            self.update_figure()
        
    def show(self):
        plt.show()

    def gps_callback(self, data):
        lat = data.latitude
        lng = data.longitude
        alt = data.altitude
        
        if self.lat0 == None:
            self.lat0   = lat
            self.lng0   = lng
        
        (self.xt, self.yt,_) = lla2flat((lat,lng,alt), (self.lat0, self.lng0), 0, 100)
        print "coordinates : (%f,%f) " % (self.xt, self.yt)
            

    def update_figure(self):
        # draw position
        if self.xt != None:
            self.ax1.plot( 0, 0, 'ko')
            self.ax1.plot( self.xt, self.yt, 'bo')
 
        # set axis
        self.ax1.set_xlim([-self.xlim, self.xlim])
        self.ax1.set_ylim([-self.ylim, self.ylim])

        # clear drawing and pause briefly
        plt.pause(0.01)
       
if __name__ == '__main__':
    # initialize the node
    rospy.init_node('gps_viewer', anonymous=True)

    try:
        node = gps_viewer()
    except rospy.ROSInterruptException:
        pass
