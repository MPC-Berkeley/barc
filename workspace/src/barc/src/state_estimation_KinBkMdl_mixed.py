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
from marvelmind_nav.msg import hedge_pos
from std_msgs.msg import Header
from numpy import eye, array, zeros, diag, unwrap, tan, cos, sin
from observers import ekf
from system_models import f_KinBkMdl_2, h_KinBkMdl_2
from tf import transformations
import math

# ***_meas are values that are used by the Kalman filters
# ***_raw are raw values coming from the sensors

class StateEst(object):
    """This class contains all variables that are read from the sensors and then passed to
    the Kalman filter."""
    # input variables
    cmd_servo = 0
    cmd_motor = 0
    cmd_t     = 0

    # IMU
    yaw_prev = 0
    yaw0 = 0            # yaw at t = 0
    yaw_meas = 0
    psiDot_meas = 0
    a_x_meas = 0
    a_y_meas = 0
    imu_updated = False

    # Velocity
    vel_meas = 0
    vel_updated = False
    vel_prev = 0
    vel_count = 0               # this counts how often the same vel measurement has been received

    # GPS
    x_meas = 0
    y_meas = 0
    gps_updated = False

    # General variables
    t0 = 0                  # Time when the estimator was started
    running = False         # bool if the car is driving

    def __init__(self):
        self.x_meas = 0

    # ecu command update
    def ecu_callback(self, data):
        self.cmd_motor = data.motor        # input motor force [N]
        self.cmd_servo = data.servo        # input steering angle [rad]
        if not self.running:               # set 'running' to True once the first command is received -> here yaw is going to be set to zero
            self.running = True

    # ultrasound gps data
    def gps_callback(self, data):
        # units: [rad] and [rad/s]
        #current_t = rospy.get_rostime().to_sec()
        self.x_meas = data.x_m
        self.y_meas = data.y_m
        self.gps_updated = True

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
        yaw = unwrap([self.yaw_prev, yaw_raw])[1]       # get smooth yaw (from beginning)
        self.yaw_prev = self.yaw_meas                   # and always use raw measured yaw for unwrapping
        # from this point on 'yaw' will be definitely unwrapped (smooth)!
        if not self.running:
            self.yaw0 = yaw              # set yaw0 to current yaw
            self.yaw_meas = 0                 # and current yaw to zero
        else:
            self.yaw_meas = yaw - self.yaw0

        # extract angular velocity and linear acceleration data
        #w_x = data.angular_velocity.x
        #w_y = data.angular_velocity.y
        w_z = data.angular_velocity.z
        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        #a_z = data.linear_acceleration.z

        self.psiDot_meas = w_z
        self.a_x_meas = a_x
        self.a_y_meas = a_y
        self.imu_updated = True

    def vel_est_callback(self, data):
        #self.vel_meas = (data.vel_fl+data.vel_fr)/2.0#data.vel_est
        if data.vel_est != self.vel_prev:
            self.vel_meas = data.vel_est
            self.vel_updated = True
            self.vel_prev = data.vel_est
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
    rospy.Subscriber('imu/data', Imu, se.imu_callback)
    rospy.Subscriber('vel_est', Vel_est, se.vel_est_callback)
    rospy.Subscriber('ecu', ECU, se.ecu_callback)
    rospy.Subscriber('hedge_pos', hedge_pos, se.gps_callback)
    state_pub_pos = rospy.Publisher('pos_info', pos_info, queue_size=1)

    # get vehicle dimension parameters
    L_f = rospy.get_param("L_a")       # distance from CoG to front axel
    L_r = rospy.get_param("L_b")       # distance from CoG to rear axel
    vhMdl = (L_f, L_r)

    # get EKF observer properties
    psi_std = rospy.get_param("state_estimation_dynamic/psi_std")   # std of measurementnoise
    v_std = rospy.get_param("state_estimation_dynamic/v_std")     # std of velocity estimation
    gps_std = rospy.get_param("state_estimation_dynamic/gps_std")   # std of gps measurements
    est_mode = rospy.get_param("state_estimation_dynamic/est_mode")  # estimation mode

    # set node rate
    loop_rate = 50
    dt = 1.0 / loop_rate
    rate = rospy.Rate(loop_rate)
    se.t0 = rospy.get_rostime().to_sec()

    z_EKF = zeros(6)                                      # x, y, psi, v, psi_drift
    P = eye(6)                                            # initial dynamics coveriance matrix

    Q = diag([0.1,0.1,1.0,0.1,0.1,0.01])
    R = diag([1.0,1.0,0.1,0.1,5.0])

    # Set up track parameters
    l = Localization()
    l.create_track()
    l.prepare_trajectory(0.06)

    d_f_hist = [0]*10       # assuming that we are running at 50Hz, array of 10 means 0.2s lag

    # Estimation variables
    (x_est, y_est) = [0]*2
    bta = 0
    v_est = 0
    psi_est = 0

    while not rospy.is_shutdown():
        # make R values dependent on current measurement (robust against outliers)
        sq_gps_dist = (se.x_meas-x_est)**2 + (se.y_meas-y_est)**2
        if se.gps_updated and sq_gps_dist < 0.5:      # if there's a new gps value:
            R[0,0] = 10.0
            R[1,1] = 10.0
        else:
            # otherwise just extrapolate measurements:
            se.x_meas = x_est + dt*(v_est*cos(psi_est+bta))
            se.y_meas = y_est + dt*(v_est*sin(psi_est+bta))
            R[0,0] = 100.0
            R[1,1] = 100.0
        if se.imu_updated:
            R[3,3] = 1.0
            R[4,4] = 5.0
        else:
            R[3,3] = 10.0
            R[4,4] = 50.0
        if se.vel_updated:
            R[2,2] = 0.1
        else:
            R[2,2] = 1.0

        se.gps_updated = False
        se.imu_updated = False
        se.vel_updated = False
        # get measurement
        y = array([se.x_meas, se.y_meas, se.yaw_meas, se.vel_meas, se.psiDot_meas])

        # define input
        d_f_hist.append(se.cmd_servo)           # this is for a 0.2 seconds delay of steering
        u = [se.cmd_motor, d_f_hist.pop(0)]

        # build extra arguments for non-linear function
        args = (u, vhMdl, dt, est_mode)

        # apply EKF and get each state estimate
        (z_EKF, P) = ekf(f_KinBkMdl_2, z_EKF, P, h_KinBkMdl_2, y, Q, R, args)

        # Read values
        (x_est, y_est, psi_est, v_est, psi_dot_est, psi_drift_est) = z_EKF           # note, r = EKF estimate yaw rate
        bta = math.atan2(L_f*tan(u[1]), L_f+L_r)
        v_y_est = L_r*psi_dot_est       # problem: results in linear dependency in system ID
        v_y_est = sin(bta)*v_est
        v_x_est = cos(bta)*v_est

        # Update track position
        l.set_pos(x_est, y_est, psi_est, v_x_est, v_y_est, psi_dot_est)
        l.find_s()
        #l.s = 0
        #l.epsi = 0
        #l.s_start = 0

        # and then publish position info
        ros_t = rospy.get_rostime()
        state_pub_pos.publish(pos_info(Header(stamp=ros_t), l.s, l.ey, l.epsi, v_est, l.s_start, l.x, l.y, l.v_x, l.v_y,
                                       l.psi, l.psiDot, se.x_meas, se.y_meas, se.yaw_meas, se.vel_meas, psi_drift_est,
                                       l.coeffX.tolist(), l.coeffY.tolist(),
                                       l.coeffTheta.tolist(), l.coeffCurvature.tolist()))

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
        state_estimation()
    except rospy.ROSInterruptException:
        pass
