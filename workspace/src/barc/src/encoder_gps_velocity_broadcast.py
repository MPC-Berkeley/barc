#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Authors: J. Noonan and Jon Gonzales
# Emails:  jpnoonan@berkeley.edu, jon.gonzales@berkeley.edu
#
# ---------------------------------------------------------------------------


import rospy
import time
import os
from sensor_msgs.msg import Imu
from barc.msg import ECU, Encoder, Z_KinBkMdl, Vel
from marvelmind_nav.msg import hedge_pos
from std_msgs.msg import Float32
from numpy import pi, cos, sin, eye, array, zeros, unwrap
from ekf import ekf
from system_models import f_KinBkMdl, h_KinBkMdl
from tf import transformations
from numpy import unwrap, sqrt, power

# from encoder
v_meas      = 0
n_FL        = 0                     # counts in the front left tire
n_FR        = 0                     # counts in the front right tire
n_BL        = 0                     # counts in the back left tire
n_BR        = 0                     # counts in the back right tire
n_FL_prev   = 0
n_FR_prev   = 0
n_BL_prev   = 0
n_BR_prev   = 0
x_prev      = 0
y_prev      = 0
r_tire      = 0.036                  # radius from tire center to perimeter along magnets [m]
dx_qrt      = 2.0*pi*r_tire/8.0     # distance along quarter tire edge [m]
t0          = time.time()
tprev_enc   = time.time()
tprev_gps   = time.time()
gps_callback_beg = True

# encoder measurement update
def enc_callback(data):
    global t0, dt_v_enc, v_meas, enc_vel_pub, tprev_enc
    global n_FL, n_FR, n_FL_prev, n_FR_prev
    global n_BL, n_BR, n_BL_prev, n_BR_prev

    n_FL = data.FL
    n_FR = data.FR
    n_BL = data.BL
    n_BR = data.BR
    # compute time elapsed
    tf = time.time()
    dt = tf - tprev_enc
    tcurr = tf - t0

    # if enough time elapse has elapsed, estimate v_x
    if dt >= dt_v_enc:
        # compute speed :  speed = distance / time
        v_FL = float(n_FL - n_FL_prev)*dx_qrt/dt
        v_FR = float(n_FR - n_FR_prev)*dx_qrt/dt
        v_BL = float(n_BL - n_BL_prev)*dx_qrt/dt
        v_BR = float(n_BR - n_BR_prev)*dx_qrt/dt

        # Modification for 3 working encoders (the back wheels are connected so only need 1 encoder between the two)
        v_meas = (v_FL + v_BL + v_BR)/3.0
        #Uncomment line below and comment line above for bench testing.
        #v_meas = v_BL

        # update old data
        n_FL_prev   = n_FL
        n_FR_prev   = n_FR
        n_BL_prev   = n_BL
        n_BR_prev   = n_BR
        tprev_enc   = time.time()
        enc_vel_pub.publish(Vel(v_meas, tcurr))
        

def gps_callback(msg):
    global gps_data_pub, t0, tprev_gps, dt_v_enc
    global x_prev, y_prev, gps_callback_beg
    x = msg.x_m
    y = msg.y_m
    tf = time.time()
    dt = tf - tprev_gps
    tcurr = tf - t0

    if (gps_callback_beg):
        gps_callback_beg = False
        x_prev = x
        y_prev = y

    # Calculate the velocity of the car from the GPS position data
    if dt >= dt_v_enc:
        v = float(sqrt(power((x - x_prev), 2) + power((y - y_prev), 2)))/dt 
        # Update data
        x_prev = x
        y_prev = y
        tprev_gps = time.time()
        gps_data_pub.publish(Vel(v, tcurr))


def main():
    global dt_v_enc
    global v_meas
    global enc_vel_pub, gps_data_pub
    # initialize node
    rospy.init_node('enc_test_node', anonymous=True)

    enc_vel_pub   = rospy.Publisher('encoder_vel', Vel, queue_size = 10) # Publishes the velocity using the encoders data
    gps_data_pub  = rospy.Publisher('gps_data', Vel, queue_size = 10) # Publishes the velocity using the GPS data
    # topic subscriptions / publications
    rospy.Subscriber('encoder', Encoder, enc_callback) 
    rospy.Subscriber('hedge_pos', hedge_pos, gps_callback) # GPS raw data

    # get encoder parameters
    dt_v_enc = rospy.get_param("enc_test_node/dt_v_enc") # time interval to compute velocity from encoders and velocity from GPS data

    # set node rate
    loop_rate   = 50
    rate        = rospy.Rate(loop_rate)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
