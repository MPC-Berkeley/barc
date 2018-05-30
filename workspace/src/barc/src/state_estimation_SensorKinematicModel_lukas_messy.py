#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

import rospy
# from Localization_helpers import Localization
from barc.msg import ECU, pos_info, Vel_est
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_pos, hedge_imu_fusion, hedge_imu_raw, hedge_pos_ang
from std_msgs.msg import Header
from numpy import eye, array, zeros, diag, unwrap, tan, cos, sin, vstack, linalg, append
from numpy import ones, polyval, delete, size
from observers import ekf
from system_models import f_SensorKinematicModel, h_SensorKinematicModel
from tf import transformations
import math
import numpy as np
import os

# ***_meas are values that are used by the Kalman filters
# ***_raw are raw values coming from the sensors

class StateEst(object):
    """This class contains all variables that are read from the sensors and then passed to
    the Kalman filter."""
    # input variables
    cmd_servo = 0.0
    cmd_motor = 0.0
    cmd_t = 0.0

    # IMU
    yaw_prev = 0.0
    yaw0 = 0.0            # yaw at t = 0
    yaw_meas = 0.0
    psiDot_meas = 0.0
    a_x_meas = 0.0
    a_y_meas = 0.0
    imu_updated = False
    att = (0.0,0.0,0.0)               # attitude
    # history data storage for debugging
    roll_raw_his = [0.0]
    pitch_raw_his = [0.0]
    yaw_raw_his = [0.0]
    yaw_his = [0.0]  # yaw after unwrap
    yaw0_his = [0.0] 
    yaw_meas_his = [0.0]
    psidot_raw_his = [0.0]
    a_x_raw_his = [0.0]
    a_y_raw_his = [0.0]
    a_x_meas_his = [0.0]
    a_y_meas_his = [0.0]
    imu_time = [0.0]
    imu_prev_time = 0.0

    # Velocity
    vel_meas = 0.0
    vel_updated = False
    vel_prev = 0.0
    vel_count = 0.0 # this counts how often the same vel measurement has been received

    # GPS
    x_meas = 0.0
    y_meas = 0.0
    x2_meas = 0.0
    y2_meas = 0.0
    gps_updated = False
    x_hist = zeros(15)
    y_hist = zeros(15)
    t_gps = zeros(15)
    c_X = array([0,0,0])
    c_Y = array([0,0,0])
    yaw_gps_meas_his = [0.0]
    yaw_gps_meas = 0.0
    # GPS IMU FUSION
    gps_x_his = [0.0]
    gps_y_his = [0.0]
    gps_time  = [0.0]

    qx_his = [0.0]
    qy_his = [0.0]
    qz_his = [0.0]
    qw_his = [0.0]
    ax_his = [0.0]
    ay_his = [0.0]
    az_his = [0.0]
    vx_his = [0.0]
    vy_his = [0.0]
    vz_his = [0.0]
    gps_imu_fusion_time = [0.0]

    # GPS IMU RAW
    acc_x_his = [0.0]
    acc_y_his = [0.0]
    acc_z_his = [0.0]
    gyro_x_his = [0.0]
    gyro_y_his = [0.0]
    gyro_z_his = [0.0]
    compass_x_his = [0.0]
    compass_y_his = [0.0]
    compass_z_his = [0.0]
    gps_imu_raw_time = [0.0]

    # real_val: true data
    x_true = 0.0
    y_true = 0.0
    vx_true = 0.0
    vy_true = 0.0
    yaw_true = 0.0
    psidot_true = 0.0
    yaw_true_his = [0.0]
    psiDot_true_his = [0.0]

    # Estimator data
    x_est = 0.0
    y_est = 0.0

    # General variables
    t0 = 0.0                # Time when the estimator was started
    running = False         # bool if the car is driving

    def __init__(self):
        self.x_meas = 0

    # ecu command update
    def ecu_callback(self, data):
        self.cmd_motor = data.motor        # input motor force [N]
        self.cmd_servo = data.servo        # input steering angle [rad]
        if not self.running:               # set 'running' to True once the first command is received -> here yaw is going to be set to zero
            self.running = True

    # TRUE CALL BACK: only for simulator
    def true_callback(self, data):
        self.x_true = data.x
        self.y_true = data.y
        self.vx_true = data.v_x
        self.vy_true = data.v_y
        self.yaw_true = data.psi
        self.psidot_true = data.psiDot
        # FOR DEBUGGING purpose
        self.yaw_true_his.append(data.psi)
        self.psiDot_true_his.append(data.psiDot)

    # ultrasound gps data
    def gps_callback(self, data):
        """This function is called when a new GPS signal is received."""
        # units: [rad] and [rad/s]
        # get current time stamp
        t_now = rospy.get_rostime().to_sec()-self.t0

        #t_msg = data.timestamp_ms/1000.0 - self.t0

        t_msg = t_now

        # if abs(t_now - t_msg) > 0.1:
        #    print "GPS: Bad synchronization - dt = %f"%(t_now-t_msg)

        # get current gps measurement 
        # self.x_meas = (data.x_m + self.x2_meas)/2
        # self.y_meas = (data.y_m + self.y2_meas)/2
        self.x_meas = data.x_m 
        self.y_meas = data.y_m
        #print "Received coordinates : (%f, %f)" % (self.x_meas, self.y_meas)

        # check if we have good measurement
        # compute distance we have travelled from previous estimate to current measurement
        # if we've travelled more than 1 m, the GPS measurement is probably junk, so ignore it
        # otherwise, store measurement, and then perform interpolation
        dist = (self.x_est-data.x_m)**2 + (self.y_est-data.y_m)**2
        if dist < 1.0:
            self.x_hist = append(self.x_hist, data.x_m)
            self.y_hist = append(self.y_hist, data.y_m)
            self.t_gps = append(self.t_gps, t_msg)

        # self.x_hist = delete(self.x_hist,0)
        # self.y_hist = delete(self.y_hist,0)
        # self.t_gps  = delete(self.t_gps,0)

        # Keep only the last second worth of coordinate data in the x_hist and y_hist buffer
        # These buffers are used for interpolation
        # without overwriting old data, the arrays would grow unbounded
        self.x_hist = self.x_hist[self.t_gps > t_now-1.0]
        self.y_hist = self.y_hist[self.t_gps > t_now-1.0]
        self.t_gps = self.t_gps[self.t_gps > t_now-1.0]
        sz = size(self.t_gps, 0)

        # perform interpolation for (x,y) as a function of time t
        # getting two function approximations x(t) and y(t)
        # 1) x(t) ~ c0x + c1x * t + c2x * t^2
        # 2) y(t) ~ c0y + c1y * t + c2y * t^2
        # c_X = [c0x c1x c2x] and c_Y = [c0y c1y c2y] 
        # use least squares to get the coefficients for this function approximation 
        # using (x,y) coordinate data from the past second (time)
        if sz > 4:
            t_matrix = vstack([self.t_gps**2, self.t_gps, ones(sz)]).T      # input matrix: [ t^2   t   1 ] 
            self.c_X = linalg.lstsq(t_matrix, self.x_hist)[0]
            self.c_Y = linalg.lstsq(t_matrix, self.y_hist)[0]
        self.gps_updated = True

        self.gps_x_his.append(data.x_m)
        self.gps_y_his.append(data.y_m)
        # self.gps_time.append(rospy.get_rostime().to_sec()-self.t0)

    # def gps2_callback(self, data):
    #     self.x2_meas = data.x_m
    #     self.y2_meas = data.y_m

    def gps_ang_callback(self, data):
        yaw = math.radians(data.angle+90)
        if not self.running:
            self.yaw_prev = 0
        else:
            self.yaw_gps_meas = np.unwrap([self.yaw_prev,yaw])[1]
            self.yaw_prev = self.yaw_gps_meas
        self.gps_time.append(rospy.get_rostime().to_sec()-self.t0)
        self.yaw_gps_meas_his.append(self.yaw_gps_meas)


    def gps_imu_fusion_callback(self,data):
        # GPS IMU FUSION
        self.gps_imu_fusion_time.append(rospy.get_rostime().to_sec()-self.t0)
        self.qx_his.append(data.qx)
        self.qy_his.append(data.qy)
        self.qz_his.append(data.qz)
        self.qw_his.append(data.qw)
        self.ax_his.append(data.ax)
        self.ay_his.append(data.ay)
        self.az_his.append(data.az)
        self.vx_his.append(data.vx)
        self.vy_his.append(data.vy)
        self.vz_his.append(data.vz)
        
    def gps_imu_raw_callback(self,data):
        # GPS IMU RAW
        self.gps_imu_raw_time.append(rospy.get_rostime().to_sec()-self.t0)
        self.acc_x_his.append(data.acc_x)
        self.acc_y_his.append(data.acc_y)
        self.acc_z_his.append(data.acc_z)
        self.gyro_x_his.append(data.gyro_x)
        self.gyro_y_his.append(data.gyro_y)
        self.gyro_z_his.append(data.gyro_z)
        self.compass_x_his.append(data.compass_x)
        self.compass_y_his.append(data.compass_y)
        self.compass_z_his.append(data.compass_z)

    # imu measurement update
    def imu_callback(self, data):
        # units: [rad] and [rad/s]
        current_t = rospy.get_rostime().to_sec()

        # get orientation from quaternion data, and convert to roll, pitch, yaw
        # extract angular velocity and linear acceleration data
        ori = data.orientation
        quaternion = (ori.x, ori.y, ori.z, ori.w)
        (roll_raw, pitch_raw, yaw_raw) = transformations.euler_from_quaternion(quaternion)
        # yaw_meas is element of [-pi,pi]
        
        # yaw = unwrap([self.yaw_prev, yaw_raw])[1]       # get smooth yaw (from beginning)
        # self.yaw_prev = self.yaw_meas                   # and always use raw measured yaw for unwrapping
        # if not self.running:
        #     self.yaw0 = yaw              # set yaw0 to current yaw
        #     self.yaw_meas = 0                 # and current yaw to zero
        # else:
        #     self.yaw_meas = yaw - self.yaw0
        
        # self.yaw_meas = yaw - self.yaw0

        if self.imu_prev_time > 0:
            self.yaw_meas += self.psiDot_meas * (current_t-self.imu_prev_time)

        w_z = data.angular_velocity.z
        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        a_z = data.linear_acceleration.z

        self.psiDot_meas = w_z
        self.a_x_meas = cos(-pitch_raw)*a_x + sin(-pitch_raw)*sin(-roll_raw)*a_y - sin(-pitch_raw)*cos(-roll_raw)*a_z
        self.a_y_meas = cos(-roll_raw)*a_y + sin(-roll_raw)*a_z

        self.att = (roll_raw,pitch_raw,yaw_raw)
        self.imu_updated = True

        # FOR DEBUGGING PURPOSE
        self.imu_time.append(current_t-self.t0)
        self.psidot_raw_his.append(w_z)
        self.a_x_raw_his.append(a_x)
        self.a_y_raw_his.append(a_y)
        self.a_x_meas_his.append(self.a_x_meas)
        self.a_y_meas_his.append(self.a_y_meas)
        self.roll_raw_his.append(roll_raw)
        self.pitch_raw_his.append(pitch_raw)
        self.yaw_raw_his.append(yaw_raw)
        # self.yaw_his.append(yaw) # yaw after unwrap()
        # self.yaw0_his.append(self.yaw0) # yaw after unwrap()
        self.yaw_meas_his.append(self.yaw_meas)

        self.imu_prev_time = current_t


    def encoder_vel_callback(self, data):
        # if data.vel_est != self.vel_prev:
        #     self.vel_meas = data.vel_est
        #     self.vel_updated = True
        #     self.vel_prev = data.vel_est
        #     self.vel_count = 0
        # else:
        #     self.vel_count = self.vel_count + 1
        #     if self.vel_count > 10:     # if 10 times in a row the same measurement
        #         self.vel_meas = 0       # set velocity measurement to zero
        #         self.vel_updated = True
        # self.vel_meas = data.vel_est
        # self.vel_updated = True

        # Average both rear speed
        v_est = (data.vel_bl + data.vel_br)/2
        if v_est != self.vel_prev:
            self.vel_meas = v_est
            self.vel_updated = True
            self.vel_prev = v_est
            self.vel_count = 0
        else:
            self.vel_count = self.vel_count + 1
            if self.vel_count > 10:     # if 10 times in a row the same measurement
                self.vel_meas = 0       # set velocity measurement to zero
                self.vel_updated = True



# state estimation node
def state_estimation():
    se = StateEst()
    # initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu/data', Imu, se.imu_callback, queue_size=1)
    rospy.Subscriber('vel_est', Vel_est, se.encoder_vel_callback, queue_size=1)
    rospy.Subscriber('ecu', ECU, se.ecu_callback, queue_size=1)
    state_pub_pos = rospy.Publisher('pos_info', pos_info, queue_size=1)

    # Simulations and Experiment will subscribe to different GPS topis
    rospy.Subscriber('hedge_imu_fusion', hedge_imu_fusion, se.gps_callback, queue_size=1)
    # rospy.Subscriber('hedge_imu_fusion_2', hedge_imu_fusion, se.gps2_callback, queue_size=1)
    rospy.Subscriber('hedge_pos_ang', hedge_pos_ang, se.gps_ang_callback, queue_size=1)


    # rospy.Subscriber('real_val', pos_info, se.true_callback, queue_size=1)
    # rospy.Subscriber('hedge_pos', hedge_pos, se.gps_callback, queue_size=1)

    # for debugging to collect the data
    rospy.Subscriber('hedge_imu_fusion', hedge_imu_fusion, se.gps_imu_fusion_callback, queue_size=1)
    rospy.Subscriber('hedge_imu_raw', hedge_imu_raw, se.gps_imu_raw_callback, queue_size=1)



    # get vehicle dimension parameters
    L_f = rospy.get_param("L_a")       # distance from CoG to front axel
    L_r = rospy.get_param("L_b")       # distance from CoG to rear axel
    vhMdl = (L_f, L_r)

    # set node rate
    loop_rate = 50
    dt = 1.0 / loop_rate
    rate = rospy.Rate(loop_rate)
    se.t0 = rospy.get_rostime().to_sec()                    # set initial time

    # z_EKF = zeros(14)                                       # x, y, psi, v, psi_drift
    # P = eye(14)                                             # initial dynamics coveriance matrix
    z_EKF = zeros(8)                                       # x, y, psi, v, psi_drift
    P = eye(8)                                             # initial dynamics coveriance matrix

    qa = 1000.0
    qp = 1000.0
    #         x, y, vx, vy, ax, ay, psi, psidot, psidrift, x, y, psi, v
    #Q = diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,dt*qp,0.01, 0.01,0.01,1.0,1.0,0.1])
    #R = diag([0.5,0.5,0.5,0.1,10.0,1.0,1.0,     5.0,5.0,0.1,0.5, 1.0, 1.0])
    
    # experiemnt
                                                                                                    # drift_1
    # Q = 0.1*diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,dt*qp, 0.001, 0.2,0.2,1.0,1.0,0.1])
    # R = 0.1*diag([100.0,100.0,1.0,1.0,1.0,100.0,100.0,     5.0,5.0,10.0,1.0, 10.0,10.0])

    # # without yaw_meas
    # Q = 0.1*diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,dt*qp, 1, 0.2,0.2,1.0,1.0,0.1])
    # R = 0.1*diag([5.0,5.0,1.0,   1.0,1000.0,1000.0,     5.0,5.0,10.0,1.0, 10.0,10.0])


    # experiemnt: single model
    #                    # x,            y,            vx,          vy,       ax,   ay,      psi,    psidot,   psidrift,
    # Q = 0.1*diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa, 1/3*dt**3*qp,  dt*qp,     0.001])
    #               # x_meas, y_meas,  vel_meas, yaw_meas, psiDot_meas, a_x_meas,  a_y_meas    
    # R = 0.1*diag([100.0,    100.0,     1.0,      1.0,     1.0,       100.0,    100.0, 10.0])

    # x,            y,            vx,          vy,       ax,   ay,      psi,    psidot,   psidrift,
    # Q = 0.1*diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa, 1/3*dt**3*qp,  dt*qp])
                  # x_meas, y_meas,  vel_meas, yaw_meas, psiDot_meas, a_x_meas,  a_y_meas    
    # R = 0.1*diag([100.0,    100.0,     1.0,               1.0,       100.0,    100.0, 10.0])

    Q = diag([0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0])
    R = diag([10.0, 10.0, 0.1, 0.1, 10.0, 10.0, 0.01])

    # R = 0.1*diag([100.0,    100.0,     1.0,      1.0,     1.0,       100.0,    100.0,   10.0, 10.0])

    # experiemnt: single model
    #           # x,   y,    vx,    vy,   ax, ay, psi,  psidot,  psidrift,
    # Q = diag([1e-2,  1e-2, 1e-2,  1e-2, 10, 10, 1e-3, 10,      1e-3 ])**2
    #           # x_meas, y_meas,  vel_meas, yaw_meas, psiDot_meas, a_x_meas,  a_y_meas    
    # R = diag([1e-2,     1e-2,    1e-2,     1e-2,     1e-2,        1.0,       1.0])**2

    # # simulation
    # Q = diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,dt*qp, 0.1, 0.2,0.2,1.0,1.0,0.1])
    # R = diag([5.0,5.0,1.0,10.0,100.0,1000.0,1000.0,     5.0,5.0,10.0,1.0, 10.0,10.0])

    # simulation single model
    # #            x,            y,            vx,          vy,       ax,   ay,      psi,         psidot,   psidrift,
    # Q = diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,   dt*qp,     0.1])
    # #         x_meas,   y_meas,  vel_meas, yaw_meas, psiDot_meas, a_x_meas,  a_y_meas    
    # R = diag([1.0,      1.0,     1.0,       10.0,       1.0,      10.0,     10.0])



    # R = diag([4*5.0,4*5.0,1.0,2*10.0,2*100.0,1000.0,1000.0,     4*5.0,4*5.0,10.0,1.0, 10.0,10.0])
    # R = diag([1*5.0,1*5.0,1.0,2*10.0,2*100.0,1000.0,1000.0,     1*5.0,1*5.0,10.0,1.0, 10.0,10.0])
    #         x,y,v,psi,psiDot,a_x,a_y, x, y, psi, v
    # R = diag([1*5.0,1*5.0,1.0,2*10.0,2*100.0,50.0,1.0,     1*5.0,1*5.0,10.0,1.0, 10.0,10.0])

    # Set up track parameters
    # l = Localization()
    # l.create_track()
    # l.create_feature_track()
    # l.create_race_track()

    d_f_hist = [0.0]*10       # assuming that we are running at 50Hz, array of 10 means 0.2s lag
    d_a_hist = [0.0]*5
    d_f_lp = 0.0
    a_lp = 0.0

    t_now = 0.0

    # Estimation variables
    (x_est, y_est, a_x_est, a_y_est, v_est_2) = [0]*5
    bta = 0.0
    v_est = 0.0
    psi_est = 0.0
    # psi_est=3.1415926
    # z_EKF[11]=psi_est

    est_counter = 0
    acc_f = 0.0
    vel_meas_est = 0.0

    psi_drift_est_his   = [0.0]
    psi_drift_est_2_his = [0.0]
    psi_est_his         = [0.0]
    psi_est_2_his       = [0.0]
    vx_est_his          = [0.0]
    vy_est_his          = [0.0]
    ax_est_his          = [0.0]
    ay_est_his          = [0.0]
    psi_dot_est_his     = [0.0]
    v2_est_his          = [0.0]
    vel_meas_his        = [0.0]
    a_his               = [0.0]
    df_his              = [0.0]
    a_lp_his            = [0.0]
    df_lp_his           = [0.0]
    estimator_time      = [0.0]
    x_est_his       = [0.0]
    y_est_his       = [0.0]
    x_est_2_his     = [0.0]
    y_est_2_his     = [0.0]
    
    msg = pos_info()

    while not rospy.is_shutdown():
        t_now = rospy.get_rostime().to_sec()-se.t0

        se.x_meas = polyval(se.c_X, t_now)
        se.y_meas = polyval(se.c_Y, t_now)
        se.gps_updated = False  
        se.imu_updated = False
        se.vel_updated = False

        # define input
        d_f_hist.append(se.cmd_servo)           # this is for a 0.2 seconds delay of steering
        # d_a_hist.append(se.cmd_motor)           # this is for a 0.2 seconds delay of steering
        
        # d_f_lp = d_f_lp + 0.5*(se.cmd_servo-d_f_lp) # low pass filter on steering
        # a_lp = a_lp + 1.0*(se.cmd_motor-a_lp)       # low pass filter on acceleration
        # u = [a_lp, d_f_hist.pop(0)]


        d_f = d_f_hist.pop(0)
        # d_f_lp = d_f_lp + 0.3*(d_f-d_f_lp) # low pass filter on steering
        # # a_lp = a_lp + 1.0*(se.cmd_motor-a_lp)       # low pass filter on acceleration
        # u = [se.cmd_motor, d_f_lp]

        u = [se.cmd_motor, d_f]     # this is with 0.2s delay for steering without low pass filter
        # u = [se.cmd_motor, se.cmd_servo]
        # u = [d_a_hist.pop(0), d_f_hist.pop(0)]     # this is with 0.2s delay for steering without low pass filter

        # u = [se.cmd_motor, se.cmd_servo]

        # u = [ 0, 0]

        bta = 0.5 * u[1]

        # print "V, V_x and V_y : (%f, %f, %f)" % (se.vel_meas,cos(bta)*se.vel_meas, sin(bta)*se.vel_meas)

        # get measurement
        y = array([se.x_meas, se.y_meas, se.vel_meas, se.yaw_meas, se.psiDot_meas, se.a_x_meas, se.a_y_meas,
                    se.x_meas, se.y_meas, se.yaw_meas, se.vel_meas, cos(bta)*se.vel_meas, sin(bta)*se.vel_meas])

        # measurement without yaw_meas from imu
        y = array([se.x_meas, se.y_meas, se.vel_meas, se.yaw_meas, se.psiDot_meas, se.a_x_meas, se.a_y_meas,
                    se.x_meas, se.y_meas, se.vel_meas, sin(bta)*se.vel_meas])

        # y = array([se.x_meas, se.y_meas, se.vel_meas, se.yaw_meas, se.psiDot_meas, se.a_x_meas, se.a_y_meas, sin(bta)*se.vel_meas])
        # y = array([se.x_meas, se.y_meas, se.vel_meas, se.yaw_meas, se.psiDot_meas, se.a_x_meas, se.a_y_meas, sin(bta)*se.vel_meas])
        y = array([se.x_meas, se.y_meas, se.vel_meas,  se.psiDot_meas, se.a_x_meas, se.a_y_meas, sin(bta)*se.vel_meas])

        

        # build extra arguments for non-linear function
        args = (u, vhMdl, dt, 0)

        # apply EKF and get each state estimate
        (z_EKF, P) = ekf(f_SensorKinematicModel, z_EKF, P, h_SensorKinematicModel, y, Q, R, args)
        # Read values
        # (x_est, y_est, v_x_est, v_y_est, a_x_est, a_y_est, psi_est, psi_dot_est, psi_drift_est,
        #     x_est_2, y_est_2, psi_est_2, v_est_2, psi_drift_est_2) = z_EKF           # note, r = EKF estimate yaw rate

        (x_est, y_est, v_x_est, v_y_est, a_x_est, a_y_est, psi_est, psi_dot_est) = z_EKF           # note, r = EKF estimate yaw rate
        psi_drift_est = 0
        # dummy
        x_est_2=0
        y_est_2=0
        psi_est_2=0
        v_est_2=0
        psi_drift_est_2=0
        

        # sections to save the data and for debugging
        estimator_time.append(t_now)
        # psi_drift_est_his.append(psi_drift_est)
        psi_drift_est_2_his.append(psi_drift_est_2)
        psi_est_his.append(psi_est)
        psi_est_2_his.append(psi_est_2)     
        vx_est_his.append(v_x_est)          
        vy_est_his.append(v_y_est)          
        ax_est_his.append(a_x_est)          
        ay_est_his.append(a_y_est)          
        psi_dot_est_his.append(psi_dot_est)     
        v2_est_his.append(v_est_2)
        vel_meas_his.append(se.vel_meas) # measurment data from encoder
        a_his.append(u[0])
        df_his.append(u[1])
        a_lp_his.append(se.cmd_motor)
        df_lp_his.append(d_f)

        x_est_his.append(x_est)
        y_est_his.append(y_est)
        x_est_2_his.append(x_est_2)
        y_est_2_his.append(y_est_2)


        se.x_est = x_est
        se.y_est = y_est
        #print "V_x and V_y : (%f, %f)" % (v_x_est, v_y_est)

        # Update track position
        # l.set_pos(x_est_2, y_est_2, psi_est_2, v_x_est, v_y_est, psi_dot_est)
        # l.set_pos(x_est,   y_est,   psi_est,   v_x_est, v_y_est, psi_dot_est)
        # l.set_pos(x_est,   y_est,  se.yaw_meas,   se.vel_meas, v_y_est, psi_dot_est)


        # Calculate new s, ey, epsi (only 12.5 Hz, enough for controller that runs at 10 Hz)
        # if est_counter%4 == 0:
        #    l.find_s()
        #l.s = 0
        #l.epsi = 0
        #l.s_start = 0

        # and then publish position info
        ros_t = rospy.get_rostime()
        msg.x = x_est
        msg.y = y_est
        msg.v_x = v_x_est
        msg.v_y = v_y_est
        msg.psi = psi_est
        msg.psiDot = psi_dot_est
        msg.u_a = se.cmd_motor
        msg.u_df = se.cmd_servo
        
        state_pub_pos.publish(msg)

        # wait
        est_counter += 1
        rate.sleep()

    homedir = os.path.expanduser("~")
    pathSave = os.path.join(homedir,"barc_debugging/estimator_output.npz")
    np.savez(pathSave,yaw_true_his      = se.yaw_true_his,
                      psiDot_true_his   = se.psiDot_true_his,
                      x_est_his     = x_est_his,
                      y_est_his     = y_est_his,
                      x_est_2_his   = x_est_2_his,
                      y_est_2_his   = y_est_2_his,
                      psi_drift_est_his     = psi_drift_est_his,
                      psi_drift_est_2_his   = psi_drift_est_2_his,
                      psi_est_his           = psi_est_his,
                      psi_est_2_his         = psi_est_2_his,
                      vx_est_his            = vx_est_his,
                      vy_est_his            = vy_est_his,
                      ax_est_his            = ax_est_his,
                      ay_est_his            = ay_est_his,
                      psi_dot_est_his       = psi_dot_est_his,  
                      v2_est_his            = v2_est_his,
                      vel_meas_his          = vel_meas_his,
                      a_his                 = a_his,
                      df_his                = df_his,
                      a_lp_his              = a_lp_his,
                      df_lp_his             = df_lp_his,
                      estimator_time        = estimator_time)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_imu.npz")
    np.savez(pathSave,psidot_raw_his    = se.psidot_raw_his,
                      a_x_raw_his       = se.a_x_raw_his,
                      a_y_raw_his       = se.a_y_raw_his,
                      a_x_meas_his      = se.a_x_meas_his,
                      a_y_meas_his      = se.a_y_meas_his,
                      roll_raw_his      = se.roll_raw_his,
                      pitch_raw_his     = se.pitch_raw_his,
                      yaw_raw_his       = se.yaw_raw_his,
                      yaw_his           = se.yaw_his,
                      yaw0_his          = se.yaw0_his,
                      yaw_meas_his      = se.yaw_meas_his,
                      imu_time          = se.imu_time)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_hedge_imu_fusion.npz")
    np.savez(pathSave,gps_x_his = se.gps_x_his,
                      gps_y_his = se.gps_y_his,
                      qx_his = se.qx_his,
                      qy_his = se.qy_his,
                      qz_his = se.qz_his,
                      qw_his = se.qw_his,
                      ax_his = se.ax_his,
                      ay_his = se.ay_his,
                      az_his = se.az_his,
                      vx_his = se.vx_his,
                      vy_his = se.vy_his,
                      vz_his = se.vz_his,
                      gps_time = se.gps_time,
                      yaw_gps_meas_his = se.yaw_gps_meas_his,
                      gps_imu_fusion_time = se.gps_imu_fusion_time)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_hedge_imu_raw.npz")
    np.savez(pathSave,acc_x_his = se.acc_x_his,
                      acc_y_his = se.acc_y_his,
                      acc_z_his = se.acc_z_his,
                      gyro_x_his = se.gyro_x_his,
                      gyro_y_his = se.gyro_y_his,
                      gyro_z_his = se.gyro_z_his,
                      compass_x_his = se.compass_x_his,
                      compass_y_his = se.compass_y_his,
                      compass_z_his = se.compass_z_his,
                      gps_imu_raw_time = se.gps_imu_raw_time)

    print "finishing saveing data"

if __name__ == '__main__':
    try:
        state_estimation()
    except rospy.ROSInterruptException:
        pass
