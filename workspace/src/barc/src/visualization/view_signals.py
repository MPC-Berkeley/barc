#!/usr/bin/env python

import rosbag
import subprocess
import os
from os import listdir
from os.path import isfile, join
import yaml
import numpy as np
import cv2
from tf import transformations
from numpy import pi
import matplotlib.pyplot as plt
from lla2flat import lla2flat
import Tkinter, tkFileDialog

rosbag_dir = os.path.expanduser("~") + '/rosbag'

# select bag file
root = Tkinter.Tk()
root.withdraw()
bag_file = tkFileDialog.askopenfilename(initialdir = rosbag_dir, title = "select bag file" , filetypes = [("Bags","*.bag")])
bag         = rosbag.Bag( bag_file )
meta_data   = bag.get_type_and_topic_info()[1]
n_msgs      = {}
topics      = meta_data.keys()

# extract number of messages transmitted on each topic
for i in range( len(topics) ):
    topic = topics[i]
    n_msgs[topic] = meta_data[topic][1]
print topics

# declare storage variables
if '/fix' in topics:
    idx_gps             = 0
    n_fix               = n_msgs['/fix']
    latitude   		    = np.zeros( n_fix )
    longitude  		    = np.zeros( n_fix )
    altitude            = np.zeros( n_fix )
    X                   = np.zeros( n_fix )
    Y                   = np.zeros( n_fix )
    t_gps               = np.zeros( n_fix )

if '/imu/data' in topics:
    idx_imu             = 0
    n_imu               = n_msgs['/imu/data']
    roll   		        = np.zeros( n_imu )
    pitch  		        = np.zeros( n_imu )
    yaw   		        = np.zeros( n_imu )
    ang_vel_x           = np.zeros( n_imu )
    ang_vel_y           = np.zeros( n_imu )
    ang_vel_z           = np.zeros( n_imu )
    lin_acc_x           = np.zeros( n_imu )
    lin_acc_y           = np.zeros( n_imu )
    lin_acc_z           = np.zeros( n_imu )
    t_imu               = np.zeros( n_imu )
    print n_imu

if '/rc_inputs' in topics:
    idx_rc              = 0
    n_rc                = n_msgs['/rc_inputs']
    throttle            = np.zeros( n_rc )
    steering            = np.zeros( n_rc )
    t_rc                = np.zeros( n_rc )

# initialize index for each measurement
t0 = -1
for topic, msg, t in bag.read_messages(topics=['/fix','/rc_inputs','/imu/data']) :
    
    # initial system time
    if t0 == -1:
        t0                  = t.secs + t.nsecs/(10.0**9)
    ts                  = t.secs + t.nsecs/(10.0**9) - t0

    if topic == '/imu/data':
        ori                 = msg.orientation
        quaternion          = (ori.x, ori.y, ori.z, ori.w)
        (r, p, y)           = transformations.euler_from_quaternion(quaternion)
        t_imu[idx_imu]      = ts 
        roll[idx_imu]        = r
        pitch[idx_imu]      = p
        yaw[idx_imu]        = y
        ang_vel_x[idx_imu]  = msg.angular_velocity.x
        ang_vel_y[idx_imu]  = msg.angular_velocity.y
        ang_vel_z[idx_imu]  = msg.angular_velocity.z
        lin_acc_x[idx_imu]  = msg.linear_acceleration.x
        lin_acc_y[idx_imu]  = msg.linear_acceleration.y
        lin_acc_z[idx_imu]  = msg.linear_acceleration.z
        idx_imu     += 1

    if topic == '/fix':
        lng                 = msg.longitude
        lat                 = msg.latitude
        alt                 = msg.altitude
        t_gps[idx_gps]      = ts 
        longitude[idx_gps]  = lng
        latitude[idx_gps]   = lat
        altitude[idx_gps]   = alt
        (X[idx], Y[idx],_)  = lla2flat((lat,lng,alt), (latitude[0], longitude[0]), 0, 100)
        idx_gps += 1

    if topic == '/rc_inputs':
        t_rc[idx_rc]        = ts 
        throttle[idx_rc]    = msg.motor
        steering[idx_rc]    = msg.servo
        idx_rc += 1

bag.close()

font = {'family':'serif',
        'color' : 'black',
        'weight': 'normal',
        'size'  : 12}
fig_sz = (14,4)


# visualize data collected
if '/fix' in topics:
    plt.figure( figsize = fig_sz)
    plt.subplot(211)
    plt.plot(t_gps, longitude)
    plt.xlabel('t [sec]')
    plt.ylabel('longitude [deg]')
    plt.subplot(212)
    plt.plot(t_gps, latitude)
    plt.xlabel('t [sec]')
    plt.ylabel('latitude [deg]')

    plt.figure()
    plt.plot(X,Y,'k*')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.grid(axis = 'both')
    plt.show()

if '/rc_inputs' in topics:
    plt.figure( figsize = fig_sz)
    plt.subplot(211)
    plt.plot(t_rc, throttle)
    plt.xlabel('t [sec]')
    plt.ylabel('throttle command')
    plt.grid(axis = 'both')
    plt.subplot(212)
    plt.plot(t_rc, steering)
    plt.xlabel('t [sec]')
    plt.ylabel('steering command')
    plt.grid(axis = 'both')
    plt.show()

if '/imu/data' in topics:
    plt.figure( figsize = fig_sz)
    plt.subplot(311)
    plt.plot(t_imu, roll)
    plt.plot(t_imu, pitch)
    plt.plot(t_imu, yaw)
    plt.xlabel('t [sec]')
    plt.ylabel('Euler angles')
    plt.grid(axis = 'both')
    plt.subplot(312)
    plt.plot(t_imu, ang_vel_x)
    plt.plot(t_imu, ang_vel_y)
    plt.plot(t_imu, ang_vel_z)
    plt.xlabel('t [sec]')
    plt.ylabel('angular velocity')
    plt.grid(axis = 'both')
    plt.subplot(313)
    plt.plot(t_imu, lin_acc_x)
    plt.plot(t_imu, lin_acc_y)
    plt.plot(t_imu, lin_acc_z)
    plt.xlabel('t [sec]')
    plt.ylabel('linear acceleration')
    plt.grid(axis = 'both')
    plt.show()

