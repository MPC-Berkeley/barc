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
from barc.msg import ECU
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from numpy import eye, array, zeros, diag, unwrap, tan, cos, sin, vstack, linalg, append, ones, polyval, delete, size, empty, linspace
from numpy import ones, polyval, delete, size, pi, sqrt
from tf import transformations
import math
import matplotlib.pyplot as plt
import numpy as np
import pylab
import matplotlib.patches as patches

global pos_info_x_vals, pos_info_y_vals
global v_vals, t_vals, psi_vals

from labs.msg import Z_DynBkMdl 

pos_info_x_vals = []
pos_info_y_vals = []

v_vals = []
t_vals = []
psi_curr = 0.0

def measurements_callback(data):
    global pos_info_x_vals, pos_info_y_vals
    global v_vals, t_vals, psi_curr
    
    pos_info_x_vals.append(data.x)
    pos_info_y_vals.append(data.y)

    v_vals.append(data.v_x)
    t_vals.append(rospy.get_rostime().to_sec())
    psi_curr = data.psi

def show():
    plt.show()

def view_trajectory():
    global pos_info_x_vals, pos_info_y_vals
    global v_vals, t_vals, psi_curr

    rospy.init_node("car_view_trajectory_node", anonymous=True)
    rospy.on_shutdown(show)

    rospy.Subscriber('z_vhcl', Z_DynBkMdl, measurements_callback)

    fig = pylab.figure()
    pylab.ion()

    vmax_ref = 1.0

    loop_rate = 100
    rate = rospy.Rate(loop_rate)

    car_dx = 1.738
    car_dy = 0.75

    car_xs_origin = [car_dx, car_dx, -car_dx, -car_dx, car_dx]
    car_ys_origin = [car_dy, -car_dy, -car_dy, car_dy, car_dy]

    car_frame = np.vstack((np.array(car_xs_origin), np.array(car_ys_origin)))
    
    edges = {'x_edge':[-10, 10], 'y_edge':[-10, 10]}
    while not rospy.is_shutdown():
        ax1 = fig.add_subplot(1, 1, 1)
        ax1.axis('equal')
        
       
        if (pos_info_x_vals and pos_info_y_vals):
            # ax1.plot(pos_info_x_vals[:len(pos_info_x_vals)-1], pos_info_y_vals[:len(pos_info_y_vals)-1], 'g-', label="Car path")
            
            x = pos_info_x_vals[len(pos_info_x_vals)-1]
            y = pos_info_y_vals[len(pos_info_y_vals)-1]
            # ax1.plot(x, y, 'gs', label="Car current pos")

            R = np.matrix([[np.cos(psi_curr), -np.sin(psi_curr)], [np.sin(psi_curr), np.cos(psi_curr)]])

            rotated_car_frame = R * car_frame

            car_xs = np.array(rotated_car_frame[0,:])[0]
            car_ys = np.array(rotated_car_frame[1,:])[0]

            front_car_segment_x = np.array([car_xs[0], car_xs[1]]) + x
            front_car_segment_y = np.array([car_ys[0], car_ys[1]]) + y

            ax1.plot(car_xs[1:] + x, car_ys[1:] + y, 'k-')
            ax1.plot(front_car_segment_x, front_car_segment_y, 'y-')
            ax1.add_patch(patches.Rectangle((x+rotated_car_frame[0,2], y+rotated_car_frame[1,2]),2*car_dx,2*car_dy,psi_curr*180/3.14))

            if x -car_dx <= edges['x_edge'][0] or x -car_dx >= edges['x_edge'][1] or y -car_dy <= edges['y_edge'][0] or y -car_dy >= edges['y_edge'][1]:
                ax1.set_xlim([x -car_dx - 10, x -car_dx + 10])
                ax1.set_ylim([y-car_dy - 10, y-car_dy + 10])
                edges['x_edge'] = [x -car_dx - 10, x -car_dx + 10]
                edges['y_edge'] = [y-car_dy - 10, y-car_dy + 10]
            else:
                ax1.set_xlim(edges['x_edge'])
                ax1.set_ylim(edges['y_edge'])

        ax1.set_title("Vehicle simulation")
        ax1.set_xlabel('x coordinate')
        ax1.set_ylabel('y coordinate')

        pylab.pause(0.0000001)
        pylab.gcf().clear()

    

if __name__ == '__main__':
    try:
        view_trajectory()
    except rospy.ROSInterruptException:
        pass
