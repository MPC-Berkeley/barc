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
from numpy import cos, sin, eye, array, zeros, diag, arctan, tan, unwrap
from observers import ekf
from system_models import f_KinBkMdl_mixed, h_KinBkMdl_mixed
from tf import transformations

# ***_meas are values that are used by the Kalman filters
# ***_raw are raw values coming from the sensors

class StateEst(object):
    """This class contains all variables that are read from the sensors and then passed to
    the Kalman filter."""
    # input variables
    cmd_servo = 0
    cmd_motor = 0

    # IMU
    yaw_prev = 0
    yaw0 = 0            # yaw at t = 0
    yaw_meas = 0
    psiDot_meas = 0
    a_x_meas = 0
    a_y_meas = 0

    # Velocity
    vel_meas = 0

    # GPS
    x_meas = 0
    y_meas = 0

    # General variables
    t0 = 0                  # Time when the estimator was started
    running = False         # bool if the car is driving

    def __init__(self):
        self.x_meas = 0

    # ecu command update
    def ecu_callback(self, data):
        self.cmd_motor = data.motor        # input motor force [N]
        self.cmd_servo = data.servo        # input steering angle [rad]
        if not self.running:                 # set 'running' to True once the first command is received -> here yaw is going to be set to zero
            self.running = True

    # ultrasound gps data
    def gps_callback(self, data):
        # units: [rad] and [rad/s]
        #current_t = rospy.get_rostime().to_sec()
        self.x_meas = data.x_m
        self.y_meas = data.y_m

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

    def vel_est_callback(self, data):
        if not data.vel_est == self.vel_meas or not self.running:        # if we're receiving a new measurement
            self.vel_meas = data.vel_est

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
    loop_rate = 25
    dt = 1.0 / loop_rate
    rate = rospy.Rate(loop_rate)
    se.t0 = rospy.get_rostime().to_sec()

    z_EKF = zeros(11)                                      # x, y, psi, v, psi_drift
    P = eye(11)                                            # initial dynamics coveriance matrix

    Q = diag([0.01,0.01,1/2*dt**2,1/2*dt**2,0.001,0.001,0.01,0.01,0.00001,0.00001,0.00001])
    R = diag([0.1,0.1,0.1,0.1,0.01,1.0,1.0])

    # Set up track parameters
    l = Localization()
    l.create_track()
    l.prepare_trajectory(0.06)

    d_f = 0

    # Estimation variables
    (x_est, y_est) = [0]*2

    while not rospy.is_shutdown():
        # make R values dependent on current measurement (robust against outliers)
        sq_gps_dist = (se.x_meas-x_est)**2 + (se.y_meas-y_est)**2
        if sq_gps_dist > 0.2:
            R[0,0] = 1+10*sq_gps_dist**2
            R[1,1] = 1+10*sq_gps_dist**2
        else:
            R[0,0] = 0.1
            R[1,1] = 0.1

        # get measurement
        y = array([se.x_meas, se.y_meas, se.vel_meas, se.yaw_meas, se.psiDot_meas, se.a_x_meas, se.a_y_meas])

        # define input
        u = [0,0]

        # build extra arguments for non-linear function
        args = (u, vhMdl, dt, est_mode)

        # apply EKF and get each state estimate
        (z_EKF, P) = ekf(f_KinBkMdl_mixed, z_EKF, P, h_KinBkMdl_mixed, y, Q, R, args)

        # Read values
        (x_est, y_est, v_x_est, v_y_est, a_x_est, a_y_est, psi_est, psi_dot_est, a_x_drift_est, a_y_drift_est, psi_drift_est) = z_EKF           # note, r = EKF estimate yaw rate

        # Update track position
        l.set_pos(x_est, y_est, psi_est, v_x_est, v_y_est, psi_dot_est)   # v = v_x
        l.find_s()
        #l.s = 0
        #l.epsi = 0
        #l.s_start = 0

        # and then publish position info
        ros_t = rospy.get_rostime()
        state_pub_pos.publish(pos_info(Header(stamp=ros_t), l.s, l.ey, l.epsi, l.v, l.s_start, l.x, l.y, l.v_x, l.v_y,
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
