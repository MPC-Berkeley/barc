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
from Localization_helpers import Localization
from barc.msg import ECU, pos_info, Vel_est, mpc_solution
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_pos
from std_msgs.msg import Header
from numpy import eye, array, zeros, diag, unwrap, tan, cos, sin, vstack, linalg, append, ones, polyval, delete, size, empty, linspace
from numpy import ones, polyval, delete, size
from observers import ekf
from system_models import f_SensorKinematicModel, h_SensorKinematicModel
from tf import transformations
import math
import matplotlib.pyplot as plt
import numpy as np
# import pylab

global gps_x_vals, gps_y_vals, gps_x_prev, gps_y_prev
global pos_info_x_vals, pos_info_y_vals, pos_info_s
global v_vals, t_vals, psi_vals
global z_x, z_y # x and y for mpcSol prediction

gps_x_vals = []
gps_y_vals = []
gps_x_prev = 0.0
gps_y_prev = 0.0

pos_info_x_vals = []
pos_info_y_vals = []
pos_info_s      = 0

v_vals = []
t_vals = []
psi_curr = 0.0

z_x = ones(11)
z_y = ones(11)

def gps_callback(data):
    global gps_x_vals, gps_y_vals, gps_x_prev, gps_y_prev

    dist = (gps_x_prev - data.x_m)**2 + (gps_y_prev - data.y_m)**2
    if dist < 1:
        gps_x_vals.append(data.x_m)
        gps_y_vals.append(data.y_m)
        gps_x_prev = data.x_m
        gps_y_prev = data.y_m
    else:
        gps_x_vals.append(gps_x_prev)
        gps_y_vals.append(gps_y_prev)


def pos_info_callback(data):
    global pos_info_x_vals, pos_info_y_vals, pos_info_s
    global v_vals, t_vals, psi_curr
    
    pos_info_x_vals.append(data.x)
    pos_info_y_vals.append(data.y)
    pos_info_s = data.s

    v_vals.append(data.v)
    t_vals.append(rospy.get_rostime().to_sec())
    psi_curr = data.psi

def mpcSol_callback(data):
    global z_x, z_y
    z_x = data.z_x
    z_y = data.z_y 
    
# def show():
#     plt.show()

def view_trajectory():

    global gps_x_vals, gps_y_vals, gps_x_prev, gps_y_prev
    global pos_info_x_vals, pos_info_y_vals, pos_info_s
    global v_vals, t_vals, psi_curr
    global z_x, z_y

    rospy.init_node("car_view_trajectory_node", anonymous=True)
    # rospy.on_shutdown(show)

    rospy.Subscriber("hedge_pos", hedge_pos, gps_callback, queue_size=1)
    rospy.Subscriber("pos_info", pos_info, pos_info_callback, queue_size=1)
    rospy.Subscriber("mpc_solution", mpc_solution, mpcSol_callback, queue_size=1)
    
    l = Localization()
    l.create_race_track()
    #l.create_circle(rad=0.8, c=array([0.0, -0.5]))

    fig = plt.figure(figsize=(10,7))
    plt.ion()
    ax1 = fig.add_subplot(1, 1, 1)
    # ax2 = fig.add_subplot(2, 1, 2)
    ax1.plot(l.nodes[0,:],l.nodes[1,:],"k--",alpha=0.4)
    ax1.plot(l.nodes_bound1[0,:],l.nodes_bound1[1,:],"r-")
    ax1.plot(l.nodes_bound2[0,:],l.nodes_bound2[1,:],"r-")
    ax1.grid('on')
    ax1.axis('equal')

    loop_rate = 50
    rate = rospy.Rate(loop_rate)

    car_dx = 0.306
    car_dy = 0.177

    car_xs_origin = [car_dx, car_dx, -car_dx, -car_dx, car_dx]
    car_ys_origin = [car_dy, -car_dy, -car_dy, car_dy, car_dy]
    car_plot, = ax1.plot(car_xs_origin,car_ys_origin,"k-")
    car_center_plot, = ax1.plot(0,0,"ko",alpha=0.4)
    pre_plot, = ax1.plot([0 for i in range(11)],[0 for i in range(11)],"b-*")
    # num = min(len(gps_x_vals),len(gps_y_vals))
    # GPS_plot, = ax1.plot(gps_x_vals, gps_y_vals, 'b-', label="GPS data path")
    num = min(len(pos_info_x_vals),len(pos_info_y_vals))
    pos_plot, = ax1.plot(pos_info_x_vals[:num], pos_info_y_vals[:num], 'g-', label="pos data path")

    t_vals_zeroed = [t - t_vals[0] for t in t_vals]
    num = min(len(t_vals_zeroed),len(v_vals))
    # v_plot, = ax2.plot(t_vals_zeroed[:num], v_vals[:num], 'm-')
    # ax2.set_ylim([0,2.5])

    plt.show()
    car_frame = np.vstack((np.array(car_xs_origin), np.array(car_ys_origin)))
    while not rospy.is_shutdown():
        if (pos_info_s>0 and pos_info_s<0.5):
            # lap switching trajectory cleaning
            gps_x_vals = [0,gps_x_vals[-1]]
            gps_y_vals = [0,gps_y_vals[-1]]
            pos_info_x_vals = [0,pos_info_x_vals[-1]]
            pos_info_y_vals = [0,pos_info_y_vals[-1]]
        # ax1 = fig.add_subplot(1, 1, 1)
        # # ax2 = fig.add_subplot(2, 1, 2)
        # ax1.plot(l.nodes[0,:],l.nodes[1,:],"r-")
        # ax1.grid('on')
        # ax1.axis('equal')
        
        # if (gps_x_vals and gps_y_vals):
        #     ax1.plot(gps_x_vals[:len(gps_x_vals)-1], gps_y_vals[:len(gps_y_vals)-1], 'b-', label="GPS data path")
        #     ax1.plot(gps_x_vals[len(gps_x_vals)-1], gps_y_vals[len(gps_y_vals)-1], 'b*',label="Car current GPS Pos")
       
        # if (pos_info_x_vals and pos_info_y_vals):
            # ax1.plot(pos_info_x_vals[:len(pos_info_x_vals)-1], pos_info_y_vals[:len(pos_info_y_vals)-1], 'g-', label="Car path")

        x = pos_info_x_vals[len(pos_info_x_vals)-1]
        y = pos_info_y_vals[len(pos_info_y_vals)-1]
        # ax1.plot(x, y, 'gs', label="Car current pos")

        R = np.matrix([[np.cos(psi_curr), -np.sin(psi_curr)], [np.sin(psi_curr), np.cos(psi_curr)]])

        rotated_car_frame = R * car_frame

        car_xs = np.array(rotated_car_frame[0,:])[0] + x
        car_ys = np.array(rotated_car_frame[1,:])[0] + y

        # front_car_segment_x = np.array([car_xs[0], car_xs[1]]) + x
        # front_car_segment_y = np.array([car_ys[0], car_ys[1]]) + y

        car_plot.set_data([car_xs[i] for i in range(5)], [car_ys[i] for i in range(5)])
        car_center_plot.set_data(x,y)
        # num = min(len(gps_x_vals),len(gps_y_vals))
        # GPS_plot.set_data(gps_x_vals, gps_y_vals) 
        num = min(len(pos_info_x_vals),len(pos_info_y_vals))
        pos_plot.set_data(pos_info_x_vals[:num], pos_info_y_vals[:num])

        # ax1.plot(front_car_segment_x, front_car_segment_y, 'y-')
        #plt.plot(np.array(car_xs_origin) + x, np.array(car_ys_origin) + y, 'k-')

        # if (v_vals):
        #     t_vals_zeroed = [t - t_vals[0] for t in t_vals]
        #     ax2.plot(t_vals_zeroed, v_vals, 'm-')

        # t_vals_zeroed = [t - t_vals[0] for t in t_vals]
        # num = min(len(t_vals_zeroed),len(v_vals))
        # v_plot.set_data(t_vals_zeroed[:num], v_vals[:num])
        # ax2.set_xlim([0,t_vals_zeroed[num-1]])
        

        # ax2.set_ylim([min(0, min(v_vals)), max(v_vals)])
        # ax1.set_title("Green: Data from POS_INFO, Blue: Data from GPS")

        # ax2.set_xlabel("Time (s)")
        # ax2.set_ylabel("Velocity (m/s)")

        pre_plot.set_data(z_x,z_y)
        # ax1.set_ylim([-10,30])
        fig.canvas.draw()
        # plt.draw()
        rate.sleep()

        # plt.gcf().clear()
    # plt.ioff()
    # plt.show()
    

if __name__ == '__main__':
    try:
        view_trajectory()
    except rospy.ROSInterruptException:
        pass
