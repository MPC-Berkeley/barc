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

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException
from barc.msg import ECU
from sensor_msgs.msg import Imu
from math import pi
from numpy import zeros, array
from numpy import unwrap
from tf import transformations
from pid import PID
import numpy as np

# pid control for constrant yaw angle 
yaw0        = 0      
read_yaw0   = False
yaw_prev    = 0      
yaw         = 0
err         = 0

############################################################
def imu_callback(data):
    # units: [rad] and [rad/s]
    global yaw0, read_yaw0, yaw_prev, yaw, err
    global yaw_prev
    
    # get orientation from quaternion data, and convert to roll, pitch, yaw
    # extract angular velocity and linear acceleration data
    ori  = data.orientation
    quaternion  = (ori.x, ori.y, ori.z, ori.w)
    (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)
    yaw         = unwrap(array([yaw_prev, yaw]), discont = pi)[1]
    yaw_prev    = yaw

    # save initial measurements
    if not read_yaw0:
        read_yaw0 = True
        yaw0    = yaw
    else:
        temp        = unwrap(array([yaw_prev, yaw]))
        yaw         = temp[1]
        yaw_prev    = yaw

#############################################################
def straight(t_i, pid, time_params,FxR_max):
    # unpack parameters
    (t_0, t_f, dt)  = time_params

    # rest
    if t_i < t_0:
        d_f         = 0
        FxR         = 0

    # start moving
    elif (t_i < t_f):
        d_f         = pid.update(yaw, dt)
        step_up     = float(t_i - t_0) / 50.0
        FxR         = np.min([ step_up, FxR_max ])

    # stop experiment
    else:
        d_f         = 0
        FxR         = 0

    return (FxR, d_f)

#############################################################
def main_auto():
    # initialize ROS node
    init_node('auto_mode', anonymous=True)
    Subscriber('imu/data', Imu, imu_callback)
    nh = Publisher('ecu', ECU, queue_size = 10)

	# set node rate
    rateHz  = 50
    rate 	= Rate(rateHz)
    dt      = 1.0 / rateHz

    # get PID parameters
    p 		= get_param("controller/p")
    i 		= get_param("controller/i")
    d 		= get_param("controller/d")
    pid     = PID(P=p, I=i, D=d)
    setReference    = False
    
    # get experiment parameters 
    t_0             = get_param("controller/t_0")     # time to start test
    t_f             = get_param("controller/t_f")     # time to end test
    FxR_max         = get_param("controller/FxR_max")
    t_params        = (t_0, t_f, dt)

    while not is_shutdown():

        # OPEN LOOP 
        if read_yaw0:
            # set reference angle
            if not setReference:
                pid.setPoint(yaw0)
                setReference    = True
                t_i             = 0.0
            # apply open loop command
            else:
                (FxR, d_f)      = straight(t_i, pid, t_params, FxR_max)
                ecu_cmd             = ECU(FxR, d_f)
                nh.publish(ecu_cmd)
                t_i += dt
	
        # wait
        rate.sleep()

#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except ROSInterruptException:
		pass
