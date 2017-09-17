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
from barc.msg import ECU, pos_info, Vel
from sensor_msgs.msg import Imu
from marvelmind_nav.msg import hedge_pos
from std_msgs.msg import Header
from numpy import eye, array, zeros, diag, unwrap, tan, cos, sin, vstack, linalg, append
from numpy import ones, polyval, delete, size
from observers import ekf
from system_models import f_SensorKinematicModel, h_SensorKinematicModel
from tf import transformations
import math

# ***_meas are values that are used by the Kalman filters
# ***_raw are raw values coming from the sensors

class StateEst(object):
    """This class contains all variables that are read from the sensors and then passed to
    the Kalman filter."""
    # input variables
    # GPS
    x_meas = 0.0
    y_meas = 0.0
    gps_updated = False
    x_hist = zeros(15)
    y_hist = zeros(15)
    t_gps = zeros(15)
    c_X = array([0,0,0])
    c_Y = array([0,0,0])
   
    # estimator data
    x_est = 0.0
    y_est = 0.0

    # General variables
    t0 = 0.0                # Time when the estimator was started
    running = False         # bool if the car is driving

    def __init__(self):
        self.x_meas = 0

    # ultrasound gps data
    def gps_callback(self, data):
        """This function is called when a new GPS signal is received."""
        # print "Inside GPS callback!!!!!"
        # units: [rad] and [rad/s]
        # get current time stamp
        t_now = rospy.get_rostime().to_sec()-self.t0
        t_msg = t_now 

        # get current gps measurement 
        self.x_meas = data.x_m
        self.y_meas = data.y_m

        # check if we have good measurement
        # compute distance we have travelled from previous estimate to current measurement
        # if we've travelled more than 1 m, the GPS measurement is probably junk, so ignore it
        # otherwise, store measurement, and then perform interpolation
        dist = (self.x_est-data.x_m)**2 + (self.y_est-data.y_m)**2
        if dist < 1.0:
            self.x_hist = append(self.x_hist, data.x_m)
            self.y_hist = append(self.y_hist, data.y_m)
            self.t_gps = append(self.t_gps, t_msg)

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

# state estimation node
def state_estimation():
    se = StateEst()
    # initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('hedge_pos', hedge_pos, se.gps_callback, queue_size=1)
    state_pub_pos = rospy.Publisher('pos_info', pos_info, queue_size=1)

    # get vehicle dimension parameters
    loop_rate = 50
    dt = 1.0 / loop_rate
    rate = rospy.Rate(loop_rate)
    se.t0 = rospy.get_rostime().to_sec()                    # set initial time

    # Set up track parameters
    l = Localization()
    l.create_track()
    t_now = 0.0

    # Estimation variables
    (x_est, y_est, a_x_est, a_y_est, v_est_2) = [0]*5
    idx = 0

    while not rospy.is_shutdown():
        t_now = rospy.get_rostime().to_sec()-se.t0

        # estimate current position based on fitted polyinomial from the GPS callback
        se.x_meas = polyval(se.c_X, t_now)
        se.y_meas = polyval(se.c_Y, t_now)
        se.gps_updated = False

        if idx%10 == 0:
            #print "current estimate (x,y) = (%f,%f)" % (se.x_meas, se.y_meas)
            print "current s := " , l.s
        
        # Update track position
        # these estimation come from the vehicle kinematic model
        l.set_pos(se.x_meas, se.y_meas, 0,0,0,0)

        # Calculate new s, ey, epsi (only 12.5 Hz, enough for controller that runs at 10 Hz)
        if idx%4 == 0:
            l.find_s()

        # and then publish position info
        ros_t = rospy.get_rostime()
        state_pub_pos.publish(pos_info(Header(stamp=ros_t), l.s, l.ey, l.epsi, 0, l.s_start, 0,0,0,0,
                                       0,0,0,0,0,0,0,
                                       0,0,0,0,0,0,0,
                                       (0,), (0,), (0,), l.coeffCurvature.tolist()))

        # wait
        idx += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        state_estimation()
    except rospy.ROSInterruptException:
        pass
