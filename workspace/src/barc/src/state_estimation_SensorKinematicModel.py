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
from Localization_helpers import Localization
from barc.msg import ECU, pos_info, Vel_est
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_pos, hedge_imu_fusion
from std_msgs.msg import Header
from numpy import eye, zeros, diag, unwrap, tan, cos, sin, vstack, linalg
from numpy import ones, polyval, delete, size
from observers import ekf
from system_models import f_SensorKinematicModel, h_SensorKinematicModel
from tf import transformations
import math
import numpy as np
import os

# ***_meas are values that are used by the Kalman filters
# ***_raw are raw values coming from the sensors


def main():
    # node initialization
    rospy.init_node("state_estimation")
    a_delay     = 0.0
    df_delay    = 0.2
    loop_rate   = 50
    dt = 1/loop_rate
    qa = 1000
    qp = 1000
    
    #                    # x,            y,            vx,          vy,       ax,       ay,      psi,    psidot
    # Q = 0.1*diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,   dt*qa,   dt*qa, 1/3*dt**3*qp,  dt*qp])
    #               # x_meas, y_meas,  vel_meas,  psiDot_meas, a_x_meas,  a_y_meas  vy_meas   
    # R = 0.1*diag([100.0,    100.0,     1.0,         1.0,       100.0,    100.0,   10.0])

    #           x,               y,             vx,         vy,             ax,          ay,      psi,              psidot
    Q = diag([1/20*dt**5*qa,    1/20*dt**5*qa,1/3*dt**3*qa, 1/3*dt**3*qa,  dt*qa,       dt*qa,  1/3*dt**3*qp,     1e8*dt*qp])
    R = diag([0.5,0.5,0.5,      1e-8,       1.0,1.0, 1.0])

    t0 = rospy.get_rostime().to_sec()
    imu = ImuClass(t0)
    gps = GpsClass(t0)
    enc = EncClass(t0)
    ecu = EcuClass(t0)
    se  = StateEst(t0,loop_rate,a_delay,df_delay,Q,R)
    
    l   = Localization()
    l.create_race_track()
    est_counter = 1
    
    while not rospy.is_shutdown():
        se.ekf(imu,gps,enc,ecu)
        
        l.set_pos(gps.x, gps.y, imu.yaw, se.vx_est, se.vy_est, se.psiDot_est)
        # l.find_s()
        if est_counter%4 == 0:

            l.find_s()

        est_counter += 1

        ros_t = rospy.get_rostime()
        se.state_pub_pos.publish(pos_info(Header(stamp=ros_t), l.s, l.ey, l.epsi, l.v, 
                                                               gps.x, gps.y, enc.v_meas, se.vy_est, 
                                                               imu.yaw, se.psiDot_est, 
                                                               se.ax_est, se.ay_est, 
                                                               ecu.a, ecu.df))

        imu.saveHistory()
        gps.saveHistory()
        enc.saveHistory()
        ecu.saveHistory()
        se.saveHistory()
        se.rate.sleep()

    homedir = os.path.expanduser("~")
    pathSave = os.path.join(homedir,"barc_debugging/estimator_output.npz")
    np.savez(pathSave,yaw_est_his       = se.yaw_est_his,
                      psiDot_est_his    = se.psiDot_est_his,
                      x_est_his         = se.x_est_his,
                      y_est_his         = se.y_est_his,
                      vx_est_his        = se.vx_est_his,
                      vy_est_his        = se.vy_est_his,
                      ax_est_his        = se.ax_est_his,
                      ay_est_his        = se.ay_est_his,
                      estimator_time    = se.time_his)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_imu.npz")
    np.savez(pathSave,psiDot_his    = imu.psiDot_his,
                      roll_his      = imu.roll_his,
                      pitch_his     = imu.pitch_his,
                      yaw_his       = imu.yaw_his,
                      ax_his        = imu.ax_his,
                      ay_his        = imu.ay_his,
                      imu_time      = imu.time_his)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_gps.npz")
    np.savez(pathSave,x_his         = gps.x_his,
                      y_his         = gps.y_his,
                      gps_time      = gps.time_his)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_enc.npz")
    np.savez(pathSave,v_fl_his          = enc.v_fl_his,
                      v_fr_his          = enc.v_fr_his,
                      v_rl_his          = enc.v_rl_his,
                      v_rr_his          = enc.v_rr_his,
                      enc_time          = enc.time_his)

    pathSave = os.path.join(homedir,"barc_debugging/estimator_ecu.npz")
    np.savez(pathSave,a_his         = ecu.a_his,
                      df_his        = ecu.df_his,
                      ecu_time      = ecu.time_his)

    print "finishing saveing state estimation data"


class StateEst(object):
    """ Object collecting  estimated state data
    Attributes:
        Estimated states:
            1.x_est     2.y_est
            3.vx_est    4.vy_est        5.v_est
            6.ax_est    7.ay_est
            8.yaw_est   9.psiDot_est    10.psiDrift_est
        Estimated states history:
            1.x_est_his     2.y_est_his
            3.vx_est_his    4.vy_est_his        5.v_est_his
            6.ax_est_his    7.ay_est_his
            8.yaw_est_his   9.psiDot_est_his    10.psiDrift_est_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """

    def __init__(self,t0,loop_rate,a_delay,df_delay,Q,R):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        dt          = 1.0 / loop_rate
        self.rate   = rospy.Rate(loop_rate)
        L_f         = rospy.get_param("L_a")       # distance from CoG to front axel
        L_r         = rospy.get_param("L_b")       # distance from CoG to rear axel
        vhMdl       = (L_f, L_r)
        self.Q      = Q
        self.R      = R
        self.P      = np.eye(np.size(Q,0)) # initializationtial covariance matrix
        self.z_EKF  = np.zeros(np.size(Q,0)) # initial state mean
        self.dt     = dt
        self.args   = (vhMdl, self.dt, 0)
        self.a_delay        = a_delay
        self.df_delay       = df_delay
        self.a_his          = [0.0]*int(a_delay/dt)
        self.df_his         = [0.0]*int(df_delay/dt)
        self.state_pub_pos  = rospy.Publisher('pos_info', pos_info, queue_size=1)
        self.t0             = t0


        self.x_est          = 0.0
        self.y_est          = 0.0
        self.vx_est         = 0.0
        self.vy_est         = 0.0
        self.v_est          = 0.0
        self.ax_est         = 0.0
        self.ay_est         = 0.0
        self.yaw_est        = 0.0
        self.psiDot_est     = 0.0
        self.psiDrift_est   = 0.0
        self.curr_time      = 0.0

        self.x_est_his          = [0.0]
        self.y_est_his          = [0.0]
        self.vx_est_his         = [0.0]
        self.vy_est_his         = [0.0]
        self.v_est_his          = [0.0]
        self.ax_est_his         = [0.0]
        self.ay_est_his         = [0.0]
        self.yaw_est_his        = [0.0]
        self.psiDot_est_his     = [0.0]
        self.time_his           = [0.0]

    # ecu command update
    def ekf(self,imu,gps,enc,ecu):
        """Do extended Kalman filter to estimate states"""
        self.curr_time = rospy.get_rostime().to_sec() - self.t0

        self.a_his.append(ecu.a)
        self.df_his.append(ecu.df)

        u = [self.a_his.pop(0), self.df_his.pop(0)]
        bta = 0.5 * u[1]

        args = (u, self.args[0], self.args[1], self.args[2])
        y = np.array([gps.x, gps.y, enc.v_meas, imu.psiDot, imu.ax, imu.ay, sin(bta)*enc.v_meas])

        (self.z_EKF, self.P) = ekf(f_SensorKinematicModel, self.z_EKF, self.P, h_SensorKinematicModel, y, self.Q, self.R, args)
        (self.x_est, self.y_est, self.vx_est, self.vy_est, self.ax_est, self.ay_est, self.yaw_est, self.psiDot_est) = self.z_EKF

    def saveHistory(self):
        
        self.time_his.append(self.curr_time)

        self.x_est_his.append(self.x_est)
        self.y_est_his.append(self.y_est)
        self.vx_est_his.append(self.vx_est)
        self.vy_est_his.append(self.vy_est)
        self.v_est_his.append(self.v_est)
        self.ax_est_his.append(self.ax_est)
        self.ay_est_his.append(self.ay_est)
        self.yaw_est_his.append(self.yaw_est)
        self.psiDot_est_his.append(self.psiDot_est)

class ImuClass(object):
    """ Object collecting GPS measurement data
    Attributes:
        Measurement:
            1.yaw 2.psiDot 3.ax 4.ay 5.roll 6.pitch
        Measurement history:
            1.yaw_his 2.psiDot_his 3.ax_his 4.ay_his 5.roll_his 6.pitch_his
        Time stamp
            1.t0 2.time_his
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """

        rospy.Subscriber('imu/data', Imu, self.imu_callback, queue_size=1)

        # Imu measurement
        self.yaw     = 0.0
        self.psiDot  = 0.0
        self.ax      = 0.0
        self.ay      = 0.0
        self.roll    = 0.0
        self.pitch   = 0.0
        
        # Imu measurement history
        self.yaw_his     = [0.0]
        self.psiDot_his  = [0.0]
        self.ax_his      = [0.0]
        self.ay_his      = [0.0]
        self.roll_his    = [0.0]
        self.pitch_his   = [0.0]
        
        # time stamp
        self.t0          = t0
        self.time_his    = [0.0]

        # Time for yawDot integration
        self.curr_time = 0.0
        self.prev_time = 0.0


    def imu_callback(self,data):
        """Unpack message from sensor, IMU"""
        
        self.curr_time = rospy.get_rostime().to_sec() - self.t0

        if self.prev_time > 0:
            self.yaw += self.psiDot * (self.curr_time-self.prev_time)

        ori = data.orientation
        quaternion = (ori.x, ori.y, ori.z, ori.w)
        (roll_raw, pitch_raw, dummy) = transformations.euler_from_quaternion(quaternion)
        self.roll   = roll_raw
        self.pitch  = pitch_raw

        w_z = data.angular_velocity.z
        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        a_z = data.linear_acceleration.z

        self.psiDot = w_z
        self.ax = cos(-pitch_raw)*a_x + sin(-pitch_raw)*sin(-roll_raw)*a_y - sin(-pitch_raw)*cos(-roll_raw)*a_z
        self.ay = cos(-roll_raw)*a_y + sin(-roll_raw)*a_z

        self.prev_time = self.curr_time


    def saveHistory(self):
        """ Save measurement data into history array"""

        self.time_his.append(self.curr_time)
        
        self.yaw_his.append(self.yaw)
        self.psiDot_his.append(self.psiDot)
        self.ax_his.append(self.ax)
        self.ay_his.append(self.ay)
        self.roll_his.append(self.roll)
        self.pitch_his.append(self.pitch)



class GpsClass(object):
    """ Object collecting GPS measurement data
    Attributes:
        Measurement:
            1.x 2.y
        Measurement history:
            1.x_his 2.y_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """

        # rospy.Subscriber('hedge_imu_fusion', hedge_imu_fusion, self.gps_callback, queue_size=1)
        rospy.Subscriber('hedge_pos', hedge_pos, self.gps_callback, queue_size=1)

        # GPS measurement
        self.x      = 0.0
        self.y      = 0.0
        
        # GPS measurement history
        self.x_his  = np.array([0.0])
        self.y_his  = np.array([0.0])
        
        # time stamp
        self.t0         = t0
        self.time_his   = np.array([0.0])
        self.curr_time  = 0.0

    def gps_callback(self,data):
        """Unpack message from sensor, GPS"""

        self.curr_time = rospy.get_rostime().to_sec() - self.t0

        self.x = data.x_m
        self.y = data.y_m

        # 1) x(t) ~ c0x + c1x * t + c2x * t^2
        # 2) y(t) ~ c0y + c1y * t + c2y * t^2
        # c_X = [c0x c1x c2x] and c_Y = [c0y c1y c2y] 
        # n_intplt = 20
        # if size(self.x_his,0) > n_intplt: # do interpolation when there is enough points
        #     x_intplt = self.x_his[-n_intplt:]
        #     y_intplt = self.y_his[-n_intplt:]
        #     t_intplt = self.time_his[-n_intplt:]
        #     t_matrix = vstack([t_intplt**2, t_intplt, ones(sz)]).T
        #     self.c_X = linalg.lstsq(t_matrix, x_intplt)[0]
        #     self.c_Y = linalg.lstsq(t_matrix, y_intplt)[0]
        #     self.x = polyval(self.c_X, self.curr_time)
        #     self.y = polyval(self.c_Y, self.curr_time)

    def saveHistory(self):
        self.time_his = np.append(self.time_his,self.curr_time)

        self.x_his = np.append(self.x_his,self.x)
        self.y_his = np.append(self.y_his,self.y)


class EncClass(object):
    """ Object collecting ENC measurement data
    Attributes:
        Measurement:
            1.v_fl 2.v_fr 3. v_rl 4. v_rr
        Measurement history:
            1.v_fl_his 2.v_fr_his 3. v_rl_his 4. v_rr_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        
        rospy.Subscriber('vel_est', Vel_est, self.enc_callback, queue_size=1)

        # ENC measurement
        self.v_fl      = 0.0
        self.v_fr      = 0.0
        self.v_rl      = 0.0
        self.v_rr      = 0.0
        self.v_meas    = 0.0
        
        # ENC measurement history
        self.v_fl_his    = [0.0]
        self.v_fr_his    = [0.0]
        self.v_rl_his    = [0.0]
        self.v_rr_his    = [0.0]
        self.v_meas_his  = [0.0]
        
        # time stamp
        self.v_count    = 0
        self.v_prev     = 0.0
        self.t0         = t0
        self.time_his   = [0.0]
        self.curr_time  = 0.0

    def enc_callback(self,data):
        """Unpack message from sensor, ENC"""

        self.curr_time = rospy.get_rostime().to_sec() - self.t0

        self.v_fl = data.vel_fl
        self.v_fr = data.vel_fr
        self.v_rl = data.vel_bl
        self.v_rr = data.vel_br

        v_est = (self.v_rl + self.v_rr)/2
        self.v_meas = v_est

        # if v_est != self.v_prev:
        #     self.v_meas = v_est
        #     self.v_prev = v_est
        #     self.v_count = 0
        # else:
        #     self.v_count = self.v_count + 1
        #     if self.v_count > 10:     # if 10 times in a row the same measurement
        #         self.v_meas = 0       # set velocity measurement to zero

    def saveHistory(self):
        self.time_his.append(self.curr_time)
        
        self.v_fl_his.append(self.v_fl)
        self.v_fr_his.append(self.v_fr)
        self.v_rl_his.append(self.v_rl)
        self.v_rr_his.append(self.v_rr)

        self.v_meas_his.append(self.v_meas)

class EcuClass(object):
    """ Object collecting CMD command data
    Attributes:
        Input command:
            1.a 2.df
        Input command history:
            1.a_his 2.df_his
        Time stamp
            1.t0 2.time_his 3.curr_time
    """
    def __init__(self,t0):
        """ Initialization
        Arguments:
            t0: starting measurement time
        """
        
        rospy.Subscriber('ecu', ECU, self.ecu_callback, queue_size=1)

        # ECU measurement
        self.a  = 0.0
        self.df = 0.0
        
        # ECU measurement history
        self.a_his  = [0.0]
        self.df_his = [0.0]
        
        # time stamp
        self.t0         = t0
        self.time_his   = [0.0]
        self.curr_time  = 0.0

    def ecu_callback(self,data):
        """Unpack message from sensor, ECU"""

        self.curr_time = rospy.get_rostime().to_sec() - self.t0

        self.a  = data.motor
        self.df = data.servo

        

    def saveHistory(self):
        self.time_his.append(self.curr_time)
        
        self.a_his.append(self.a)
        self.df_his.append(self.df)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




# # state estimation node
# def state_estimation():
#     se = StateEst()
#     # initialize node
#     rospy.init_node('state_estimation', anonymous=True)

#     # topic subscriptions / publications
#     rospy.Subscriber('imu/data', Imu, se.imu_callback, queue_size=1)
#     rospy.Subscriber('vel_est', Vel_est, se.encoder_vel_callback, queue_size=1)
#     rospy.Subscriber('ecu', ECU, se.ecu_callback, queue_size=1)
#     state_pub_pos = rospy.Publisher('pos_info', pos_info, queue_size=1)

#     # Simulations and Experiment will subscribe to different GPS topis
#     rospy.Subscriber('hedge_imu_fusion', hedge_imu_fusion, se.gps_callback, queue_size=1)
#     # rospy.Subscriber('hedge_imu_fusion_2', hedge_imu_fusion, se.gps2_callback, queue_size=1)
#     rospy.Subscriber('hedge_pos_ang', hedge_pos_ang, se.gps_ang_callback, queue_size=1)


#     # rospy.Subscriber('real_val', pos_info, se.true_callback, queue_size=1)
#     # rospy.Subscriber('hedge_pos', hedge_pos, se.gps_callback, queue_size=1)

#     # for debugging to collect the data
#     rospy.Subscriber('hedge_imu_fusion', hedge_imu_fusion, se.gps_imu_fusion_callback, queue_size=1)
#     rospy.Subscriber('hedge_imu_raw', hedge_imu_raw, se.gps_imu_raw_callback, queue_size=1)



#     # get vehicle dimension parameters
#     L_f = rospy.get_param("L_a")       # distance from CoG to front axel
#     L_r = rospy.get_param("L_b")       # distance from CoG to rear axel
#     vhMdl = (L_f, L_r)

#     # set node rate
#     loop_rate = 50
#     dt = 1.0 / loop_rate
#     rate = rospy.Rate(loop_rate)
#     se.t0 = rospy.get_rostime().to_sec()                    # set initial time

#     z_EKF = zeros(14)                                       # x, y, psi, v, psi_drift
#     P = eye(14)                                             # initial dynamics coveriance matrix
#     # z_EKF = zeros(9)                                       # x, y, psi, v, psi_drift
#     # P = eye(9)                                             # initial dynamics coveriance matrix

#     qa = 1000.0
#     qp = 1000.0
#     #         x, y, vx, vy, ax, ay, psi, psidot, psidrift, x, y, psi, v
#     #Q = diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,dt*qp,0.01, 0.01,0.01,1.0,1.0,0.1])
#     #R = diag([0.5,0.5,0.5,0.1,10.0,1.0,1.0,     5.0,5.0,0.1,0.5, 1.0, 1.0])
    
#     # experiemnt
#                                                                                                     # drift_1
#     Q = 0.1*diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,dt*qp, 0.001, 0.2,0.2,1.0,1.0,0.1])
#     R = 0.1*diag([100.0,100.0,1.0,1.0,1.0,100.0,100.0,     5.0,5.0,10.0,1.0, 10.0,10.0])

#     # # without yaw_meas
#     # Q = 0.1*diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,dt*qp, 1, 0.2,0.2,1.0,1.0,0.1])
#     # R = 0.1*diag([5.0,5.0,1.0,   1.0,1000.0,1000.0,     5.0,5.0,10.0,1.0, 10.0,10.0])


#     # experiemnt: single model
#     #                    # x,            y,            vx,          vy,       ax,   ay,      psi,    psidot,   psidrift,
#     # Q = 0.1*diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa, 1/3*dt**3*qp,  dt*qp,     0.001])
#     #               # x_meas, y_meas,  vel_meas, yaw_meas, psiDot_meas, a_x_meas,  a_y_meas    
#     # # R = 0.1*diag([100.0,    100.0,     1.0,      1.0,     1.0,       100.0,    100.0, 10.0])
#     # R = 0.1*diag([100.0,    100.0,     1.0,      1.0,     1.0,       100.0,    100.0,   10.0, 10.0])

#     # experiemnt: single model
#     #           # x,   y,    vx,    vy,   ax, ay, psi,  psidot,  psidrift,
#     # Q = diag([1e-2,  1e-2, 1e-2,  1e-2, 10, 10, 1e-3, 10,      1e-3 ])**2
#     #           # x_meas, y_meas,  vel_meas, yaw_meas, psiDot_meas, a_x_meas,  a_y_meas    
#     # R = diag([1e-2,     1e-2,    1e-2,     1e-2,     1e-2,        1.0,       1.0])**2

#     # # simulation
#     # Q = diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,dt*qp, 0.1, 0.2,0.2,1.0,1.0,0.1])
#     # R = diag([5.0,5.0,1.0,10.0,100.0,1000.0,1000.0,     5.0,5.0,10.0,1.0, 10.0,10.0])

#     # simulation single model
#     # #            x,            y,            vx,          vy,       ax,   ay,      psi,         psidot,   psidrift,
#     # Q = diag([1/20*dt**5*qa,1/20*dt**5*qa,1/3*dt**3*qa,1/3*dt**3*qa,dt*qa,dt*qa,1/3*dt**3*qp,   dt*qp,     0.1])
#     # #         x_meas,   y_meas,  vel_meas, yaw_meas, psiDot_meas, a_x_meas,  a_y_meas    
#     # R = diag([1.0,      1.0,     1.0,       10.0,       1.0,      10.0,     10.0])



#     # R = diag([4*5.0,4*5.0,1.0,2*10.0,2*100.0,1000.0,1000.0,     4*5.0,4*5.0,10.0,1.0, 10.0,10.0])
#     # R = diag([1*5.0,1*5.0,1.0,2*10.0,2*100.0,1000.0,1000.0,     1*5.0,1*5.0,10.0,1.0, 10.0,10.0])
#     #         x,y,v,psi,psiDot,a_x,a_y, x, y, psi, v
#     # R = diag([1*5.0,1*5.0,1.0,2*10.0,2*100.0,50.0,1.0,     1*5.0,1*5.0,10.0,1.0, 10.0,10.0])

#     # Set up track parameters
#     l = Localization()
#     # l.create_track()
#     # l.create_feature_track()
#     l.create_race_track()

#     d_f_hist = [0.0]*10       # assuming that we are running at 50Hz, array of 10 means 0.2s lag
#     d_a_hist = [0.0]*5
#     d_f_lp = 0.0
#     a_lp = 0.0

#     t_now = 0.0

#     # Estimation variables
#     (x_est, y_est, a_x_est, a_y_est, v_est_2) = [0]*5
#     bta = 0.0
#     v_est = 0.0
#     psi_est = 0.0
#     # psi_est=3.1415926
#     # z_EKF[11]=psi_est

#     est_counter = 0
#     acc_f = 0.0
#     vel_meas_est = 0.0

#     psi_drift_est_his   = [0.0]
#     psi_drift_est_2_his = [0.0]
#     psi_est_his         = [0.0]
#     psi_est_2_his       = [0.0]
#     vx_est_his          = [0.0]
#     vy_est_his          = [0.0]
#     ax_est_his          = [0.0]
#     ay_est_his          = [0.0]
#     psi_dot_est_his     = [0.0]
#     v2_est_his          = [0.0]
#     vel_meas_his        = [0.0]
#     a_his               = [0.0]
#     df_his              = [0.0]
#     a_lp_his            = [0.0]
#     df_lp_his           = [0.0]
#     estimator_time      = [0.0]
#     x_est_his       = [0.0]
#     y_est_his       = [0.0]
#     x_est_2_his     = [0.0]
#     y_est_2_his     = [0.0]
 
#     while not rospy.is_shutdown():
#         t_now = rospy.get_rostime().to_sec()-se.t0

#         se.x_meas = polyval(se.c_X, t_now)
#         se.y_meas = polyval(se.c_Y, t_now)
#         se.gps_updated = False  
#         se.imu_updated = False
#         se.vel_updated = False

#         # define input
#         d_f_hist.append(se.cmd_servo)           # this is for a 0.2 seconds delay of steering
#         # d_a_hist.append(se.cmd_motor)           # this is for a 0.2 seconds delay of steering
        
#         # d_f_lp = d_f_lp + 0.5*(se.cmd_servo-d_f_lp) # low pass filter on steering
#         # a_lp = a_lp + 1.0*(se.cmd_motor-a_lp)       # low pass filter on acceleration
#         # u = [a_lp, d_f_hist.pop(0)]


#         d_f = d_f_hist.pop(0)
#         # d_f_lp = d_f_lp + 0.3*(d_f-d_f_lp) # low pass filter on steering
#         # # a_lp = a_lp + 1.0*(se.cmd_motor-a_lp)       # low pass filter on acceleration
#         # u = [se.cmd_motor, d_f_lp]

#         u = [se.cmd_motor, d_f]     # this is with 0.2s delay for steering without low pass filter

#         # u = [d_a_hist.pop(0), d_f_hist.pop(0)]     # this is with 0.2s delay for steering without low pass filter

#         # u = [se.cmd_motor, se.cmd_servo]

#         # u = [ 0, 0]

#         bta = 0.5 * u[1]

#         # print "V, V_x and V_y : (%f, %f, %f)" % (se.vel_meas,cos(bta)*se.vel_meas, sin(bta)*se.vel_meas)

#         # get measurement
#         y = array([se.x_meas, se.y_meas, se.vel_meas, se.yaw_meas, se.psiDot_meas, se.a_x_meas, se.a_y_meas,
#                     se.x_meas, se.y_meas, se.yaw_meas, se.vel_meas, cos(bta)*se.vel_meas, sin(bta)*se.vel_meas])

#         # # measurement without yaw_meas from imu
#         # y = array([se.x_meas, se.y_meas, se.vel_meas, se.yaw_meas, se.psiDot_meas, se.a_x_meas, se.a_y_meas,
#         #             se.x_meas, se.y_meas, se.vel_meas, cos(bta)*se.vel_meas, sin(bta)*se.vel_meas])

#         # y = array([se.x_meas, se.y_meas, se.vel_meas, se.yaw_meas, se.psiDot_meas, se.a_x_meas, se.a_y_meas, sin(bta)*se.vel_meas])
#         # y = array([se.x_meas, se.y_meas, se.vel_meas, se.yaw_meas, se.psiDot_meas, se.a_x_meas, se.a_y_meas, cos(bta)*se.vel_meas, sin(bta)*se.vel_meas])

        

#         # build extra arguments for non-linear function
#         args = (u, vhMdl, dt, 0)

#         # apply EKF and get each state estimate
#         (z_EKF, P) = ekf(f_SensorKinematicModel, z_EKF, P, h_SensorKinematicModel, y, Q, R, args)
#         # Read values
#         (x_est, y_est, v_x_est, v_y_est, a_x_est, a_y_est, psi_est, psi_dot_est, psi_drift_est,
#             x_est_2, y_est_2, psi_est_2, v_est_2, psi_drift_est_2) = z_EKF           # note, r = EKF estimate yaw rate

#         # (x_est, y_est, v_x_est, v_y_est, a_x_est, a_y_est, psi_est, psi_dot_est, psi_drift_est) = z_EKF           # note, r = EKF estimate yaw rate
#         # dummy
#         x_est_2=0
#         y_est_2=0
#         psi_est_2=0
#         v_est_2=0
#         psi_drift_est_2=0
        

#         # sections to save the data and for debugging
#         estimator_time.append(t_now)
#         psi_drift_est_his.append(psi_drift_est)
#         psi_drift_est_2_his.append(psi_drift_est_2)
#         psi_est_his.append(psi_est)
#         psi_est_2_his.append(psi_est_2)     
#         vx_est_his.append(v_x_est)          
#         vy_est_his.append(v_y_est)          
#         ax_est_his.append(a_x_est)          
#         ay_est_his.append(a_y_est)          
#         psi_dot_est_his.append(psi_dot_est)     
#         v2_est_his.append(v_est_2)
#         vel_meas_his.append(se.vel_meas) # measurment data from encoder
#         a_his.append(u[0])
#         df_his.append(u[1])
#         a_lp_his.append(se.cmd_motor)
#         df_lp_his.append(d_f)

#         x_est_his.append(x_est)
#         y_est_his.append(y_est)
#         x_est_2_his.append(x_est_2)
#         y_est_2_his.append(y_est_2)


#         se.x_est = x_est
#         se.y_est = y_est
#         #print "V_x and V_y : (%f, %f)" % (v_x_est, v_y_est)

#         # Update track position
#         # l.set_pos(x_est_2, y_est_2, psi_est_2, v_x_est, v_y_est, psi_dot_est)
#         l.set_pos(x_est,   y_est,   psi_est,   v_x_est, v_y_est, psi_dot_est)
#         # l.set_pos(x_est,   y_est,  se.yaw_meas,   se.vel_meas, v_y_est, psi_dot_est)


#         # Calculate new s, ey, epsi (only 12.5 Hz, enough for controller that runs at 10 Hz)
#         if est_counter%4 == 0:
#             l.find_s()
#         #l.s = 0
#         #l.epsi = 0
#         #l.s_start = 0

#         # and then publish position info
#         ros_t = rospy.get_rostime()
#         state_pub_pos.publish(pos_info(Header(stamp=ros_t), l.s, l.ey, l.epsi, v_est_2, l.x, l.y, l.v_x, l.v_y,
#                                        l.psi, l.psiDot, se.x_meas, se.y_meas, se.yaw_meas, se.vel_meas, se.psiDot_meas,
#                                        psi_drift_est, a_x_est, a_y_est, se.a_x_meas, se.a_y_meas, se.cmd_motor, se.cmd_servo,
#                                        (0,), (0,), (0,), l.curv_curr))

#         # wait
#         est_counter += 1
#         rate.sleep()
