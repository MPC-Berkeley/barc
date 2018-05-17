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
global v_vals, t_vals, psi_curr, psi_raw
global z_x, z_y, SS_x, SS_y, z_vx, SS_vx, z_s, SS_s, z_fore_x, z_fore_y, z_iden_x, z_iden_y # x and y for mpcSol prediction

gps_x_vals = []
gps_y_vals = []
gps_x_prev = 0.0
gps_y_prev = 0.0

pos_info_x_vals = [0]
pos_info_y_vals = [0]
pos_info_s      = 0

v_vals = []
t_vals = []
psi_curr = 0.0
psi_raw = 0.0

z_x = ones(11)
z_y = ones(11)
z_fore_x = ones(11)
z_fore_y = ones(11)

SS_x = zeros(20)
SS_y = zeros(20)

z_vx = zeros(11)
z_s = zeros(11)
SS_vx = zeros(20)
SS_s = zeros(20)

z_iden_x = zeros(30)
z_iden_y = zeros(30)

def gps_callback(data):
    global gps_x_vals, gps_y_vals, gps_x_prev, gps_y_prev, z_vx, SS_vx

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
    global v_vals, t_vals, psi_curr, psi_raw
    
    pos_info_x_vals.append(data.x)
    pos_info_y_vals.append(data.y)
    pos_info_s = data.s

    v_vals.append(data.v)
    t_vals.append(rospy.get_rostime().to_sec())
    psi_curr = data.psi
    psi_raw = data.psi_raw

def mpcSol_callback(data):
    global z_x, z_y, SS_x, SS_y, z_vx, SS_vx, z_s, SS_s, z_fore_x, z_fore_y, z_iden_x, z_iden_y
    z_x = data.z_x
    z_y = data.z_y 
    SS_x = data.SS_x
    SS_y = data.SS_y
    z_vx = data.z_vx
    SS_vx = data.SS_vx
    z_s = data.z_s
    SS_s = data.SS_s
    z_fore_x = data.z_fore_x
    z_fore_y = data.z_fore_y
    z_iden_x = data.z_iden_x
    z_iden_y = data.z_iden_y

# def show():
#     plt.show()

def view_trajectory():

    global gps_x_vals, gps_y_vals, gps_x_prev, gps_y_prev
    global pos_info_x_vals, pos_info_y_vals, pos_info_s
    global v_vals, t_vals, psi_curr, psi_raw
    global z_x, z_y, SS_x, SS_y, z_vx, SSvx, z_s, SS_s, z_fore_x, z_fore_y, z_iden_x, z_iden_y

    rospy.init_node("car_view_trajectory_node", anonymous=True)
    # rospy.on_shutdown(show)

    rospy.Subscriber("hedge_pos", hedge_pos, gps_callback, queue_size=1)
    rospy.Subscriber("pos_info", pos_info, pos_info_callback, queue_size=1)
    rospy.Subscriber("mpc_solution", mpc_solution, mpcSol_callback, queue_size=1)

    # FLAGS FOR PLOTTING
    PRE_FLAG = False
    SS_FLAG  = True
    FORE_FLAG= False
    IDEN_FLAG= True
    GPS_FLAG = True
    YAW_FLAG = True
    
    l = Localization()
    # l.create_feature_track()
    l.create_race_track()

    fig = plt.figure(figsize=(10,7))
    plt.ion()
    ax1 = fig.add_subplot(1, 1, 1)
    # ax2 = fig.add_subplot(2, 1, 2)
    ax1.plot(l.nodes[0,:],l.nodes[1,:],"k--",alpha=0.4)
    ax1.plot(l.nodes_bound1[0,:],l.nodes_bound1[1,:],"r-")
    ax1.plot(l.nodes_bound2[0,:],l.nodes_bound2[1,:],"r-")
    ax1.grid('on')
    ax1.axis('equal')
    # ax1.set_ylim([-5.5,1])

    loop_rate = 50
    rate = rospy.Rate(loop_rate)

    car_dx = 0.3
    car_dy = 0.12

    car_xs_origin = [car_dx, car_dx, -car_dx, -car_dx, car_dx]
    car_ys_origin = [car_dy, -car_dy, -car_dy, car_dy, car_dy]
    car_plot, = ax1.plot(car_xs_origin,car_ys_origin,"k-")

    if YAW_FLAG:
        yaw_raw_plot, = ax1.plot(car_xs_origin,car_ys_origin,"r-")

    car_center_plot, = ax1.plot(0,0,"ko",alpha=0.4)

    if PRE_FLAG:
        pre_plot, = ax1.plot([0 for i in range(11)],[0 for i in range(11)],"b-*")
    
    if SS_FLAG:
        SS_plot, = ax1.plot([0 for i in range(20)],[0 for i in range(20)],"ro",alpha=0.2)
    
    if FORE_FLAG:
        fore_plot, = ax1.plot([0 for i in range(11)],[0 for i in range(11)],"k.")

    if IDEN_FLAG:    
        iden_plot, = ax1.plot([0 for i in range(30)],[0 for i in range(30)],"go",alpha=0.3)

    if GPS_FLAG:
        num = min(len(gps_x_vals),len(gps_y_vals))
        GPS_plot, = ax1.plot(gps_x_vals, gps_y_vals, 'b-', label="GPS data path")

    num = min(len(pos_info_x_vals),len(pos_info_y_vals))
    pos_plot, = ax1.plot(pos_info_x_vals[:num], pos_info_y_vals[:num], 'g-', label="pos data path")

    # ax2 PLOT CHOICE 1: V_X HISTORY PLOT
    # t_vals_zeroed = [t - t_vals[0] for t in t_vals]
    # num = min(len(t_vals_zeroed),len(v_vals))
    # v_plot, = ax2.plot(t_vals_zeroed[:num], v_vals[:num], 'm-')
    # ax2.set_ylim([0,2.5])

    # ax2 PLOT CHOICE 2: V_X SS AND PREDICTION PLOT
    # v_plot, = ax2.plot(z_s,z_vx,"b-")
    # SS_v_plot, = ax2.plot(SS_s,SS_vx,"k*")
    # ax2.set_xlim([min(z_s), max(SS_s)])
    # ax2.set_ylim([min(min(z_vx),min(SS_vx)), max(max(z_vx),max(SS_vx))])

    plt.show()
    car_frame = np.vstack((np.array(car_xs_origin), np.array(car_ys_origin)))

    counter_buffer = 600
    counter = 1
    while not rospy.is_shutdown():
        # if counter < counter_buffer:
        if (pos_info_s>0 and pos_info_s<0.5):
            # lap switching trajectory cleaning
            gps_x_vals = [0,gps_x_vals[-1]]
            gps_y_vals = [0,gps_y_vals[-1]]
            pos_info_x_vals = [0,pos_info_x_vals[-1]]
            pos_info_y_vals = [0,pos_info_y_vals[-1]]

            # gps_x_vals = [0,0]
            # gps_y_vals = [0,0]
            # pos_info_x_vals = [0,0]
            # pos_info_y_vals = [0,0]
        

        x = pos_info_x_vals[len(pos_info_x_vals)-1]
        y = pos_info_y_vals[len(pos_info_y_vals)-1]
        # ax1.plot(x, y, 'gs', label="Car current pos")

        R = np.matrix([[np.cos(psi_curr), -np.sin(psi_curr)], [np.sin(psi_curr), np.cos(psi_curr)]])

        rotated_car_frame = R * car_frame
        car_xs = np.array(rotated_car_frame[0,:])[0] + x
        car_ys = np.array(rotated_car_frame[1,:])[0] + y

        car_plot.set_data([car_xs[i] for i in range(5)], [car_ys[i] for i in range(5)])
        car_center_plot.set_data(x,y)

        if YAW_FLAG:
            R_yaw_raw = np.matrix([[np.cos(psi_raw), -np.sin(psi_raw)], [np.sin(psi_raw), np.cos(psi_raw)]])
            yaw_raw_rotated_car_frame = R_yaw_raw * car_frame
            car_xs = np.array(rotated_car_frame[0,:])[0] + x
            car_ys = np.array(rotated_car_frame[1,:])[0] + y
            yaw_raw_plot.set_data([car_xs[i] for i in range(5)], [car_ys[i] for i in range(5)])

        if GPS_FLAG:
            num = min(len(gps_x_vals),len(gps_y_vals))
            GPS_plot.set_data(gps_x_vals[:num], gps_y_vals[:num])

        num = min(len(pos_info_x_vals),len(pos_info_y_vals))
        pos_plot.set_data(pos_info_x_vals[:num], pos_info_y_vals[:num])

        # ax2 PLOT CHOICE 1: V_X HISTORY PLOT
        # v_plot.set_data(z_vx)

        # ax2 PLOT CHOICE 2: V_X SS AND PREDICTION PLOT
        # v_plot.set_data(z_s,z_vx)
        # SS_v_plot.set_data(SS_s,SS_vx)
        # if len(z_s) == 0:
        #     pass
        # else:
        #     ax2.set_xlim([min(z_s), max(SS_s)])
        #     ax2.set_ylim([min(min(z_vx),min(SS_vx)), max(max(z_vx),max(SS_vx))])
        if PRE_FLAG:
            pre_plot.set_data(z_x,z_y)

        if FORE_FLAG:
            fore_plot.set_data(z_fore_x,z_fore_y)
        
        if IDEN_FLAG:
            iden_plot.set_data(z_iden_x,z_iden_y)
        
        if SS_FLAG:
            SS_plot.set_data(SS_x,SS_y)
        
        fig.canvas.draw()
        counter+=1
        # print(counter)
        rate.sleep()


    # plt.ioff()
    # plt.show()
    

if __name__ == '__main__':
    try:
        view_trajectory()
    except rospy.ROSInterruptException:
        pass
