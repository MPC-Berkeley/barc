#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu)  Development of the web-server app Dator was
# based on an open source project by Bruce Wootton, with contributions from
# Kiet Lam (kiet.lam@berkeley.edu)
# ---------------------------------------------------------------------------

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from data_service.msg import TimeData
from math import pi,sin
import time
import serial
from numpy import array, dot, cos, tan, arctan, ceil, array, float32, zeros, arcsin
from input_map import angle_2_servo, servo_2_angle
from manuevers import TestSettings, CircularTest, Straight
from manuevers import SineSweep, DoubleLaneChange, CoastDown, SingleTurn 
import scipy.io as sio
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from pid import PID

# global constants
g = 9.81

# pid control for constrant yaw angle 
# -> d(yaw)/dt = 0 means no turning => straight path
yaw0    = 0                 # counts in the front left tire
yaw     = 0
read_yaw0 = False
w_z 	  = 0                 # counts in the front right tire
err     = 0
def angle_callback(data):
    global yaw, yaw0, w_z, err, read_yaw0

    # save initial measurements
    if not read_yaw0:
        yaw0 = data.x
        read_yaw0 = True

    # compute deviation from original yaw angle
    yaw = data.x
    err = yaw - yaw0


#############################################################
# get estimate of x_hat = [beta , r, v_x]
x_hat = zeros(3)
def updateState_callback(data):
	global x_hat

	# update fields
	x_hat[0] = data.x 		# v_x       longitudinal velocity estimate 
	x_hat[1] = data.y 		# v_y       lateral velocity estimate
	x_hat[2] = data.z		# r         yaw rate estimate

#############################################################
def LQR_drift(z_eq, K_LQR, vhMdl,TrMdl, F_ext, u_nominal, offsets):

	# get current state estimate
    global x_hat
    v_x 	= x_hat[0]
    v_y 	= x_hat[1]
    r 	    = x_hat[2]

    # get the vehicle model, tire model, external force model
    (L_a, L_b, m, _)    = vhMdl
    (B,C,mu)            = TrMdl
    (a0, Ff)            = F_ext
    Fn                  = mu*m*g/2

    # unpack the offsets
    d_f_offset, motor_offset, motor_min = offsets
    
	# compute slip angle beta
    beta    = arctan(v_y / v_x)

    # compute error state
    z_hat       = array([beta, r, v_x]).reshape(-1,1)
    z_error     = z_hat - z_eq
    
    # compute input, u = [ FxR_desired, FyR_desired] 
    u       = dot(K_LQR, z_error) + u_nominal
    u       = u.flatten()
    (FyF, FxR)  = u

    # saturate the input lateral force
    FyF     = min(Fn, max(-Fn, FyF))

    # get map from FxR to u_motor 
    motorCMD    = ceil(  (FxR/m + Ff + a0*v_x**2) / 0.3 + 95  ) + motor_offset 
    motorCMD    = max(int(motor_min), motorCMD)          # lower bound for motor command

    # get map from FyF to steering angle
    a_f     = tan(  arcsin(-(2*FyF / (m*g*mu)) ) / C ) / B
    d_f     = arctan((v_y + L_a*r)/v_x) - a_f
    d_f     += d_f_offset
    d_f_deg = min(15, max(-10, d_f*180/pi ))    # saturation on steering angle
    servoCMD    = angle_2_servo( d_f_deg )   

    return (motorCMD, servoCMD, d_f_deg*pi/180)

#############################################################
def main_auto():
	# initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    rospy.Subscriber('state_estimate', Vector3, updateState_callback)
    rospy.Subscriber('angle_info', Vector3, angle_callback)
    nh      = rospy.Publisher('ecu_cmd', Vector3, queue_size = 10)
    debug   = rospy.Publisher('debug', numpy_msg(Floats), queue_size=10)

	# set node rate
    rateHz  = 50
    rate 	= rospy.Rate(rateHz)
    dt      = 1.0 / rateHz
    t_i     = 0

    # get vehicle properties
    L_a = rospy.get_param("state_estimation/L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("state_estimation/L_b")       # distance from CoG to rear axel
    m   = rospy.get_param("state_estimation/m")         # mass of vehicle
    I_z = rospy.get_param("state_estimation/I_z")       # moment of inertia about z-axis
    vhMdl   = (L_a, L_b, m, I_z)

    # get tire model
    B   = rospy.get_param("state_estimation/B")
    C   = rospy.get_param("state_estimation/C")
    mu  = rospy.get_param("state_estimation/mu")
    TrMdl = [B,C,mu]

    # get external force model
    a0  = rospy.get_param("state_estimation/air_drag_coeff")
    Ff  = rospy.get_param("state_estimation/Ff")
    F_ext   = (a0, Ff)
	
    # use simple pid control to keep steering straight initially
    p 		= rospy.get_param("controller/p")
    i 		= rospy.get_param("controller/i")
    d 		= rospy.get_param("controller/d")
    pid     = PID(P=p, I=i, D=d)

    # get LQR gain matrix and P matrix data
    data_path   = '/home/odroid/barc/workspace/src/barc/control/'
    data_file   = 'con_para.mat'
    lqr_data    = sio.loadmat(data_path + data_file)['con_para'][0][0]
    z_eq        = lqr_data['z_eq']
    u_eq        = lqr_data['U_eq']
    K_LQR       = lqr_data['K']
    P           = lqr_data['P']
    gamma       = 3*(lqr_data['r'].flatten()[0])

    # get LQR offset data
    d_f_offset      = rospy.get_param("controller/d_f_offset")*pi/180
    motor_offset    = rospy.get_param("controller/motor_offset")
    motor_min       = rospy.get_param("controller/motor_min")
    offsets         = [d_f_offset, motor_offset, motor_min]

    # specify test and test options
    experiment_opt    = { 0 : CircularTest,
                          1 : Straight,
		    		      2 : SineSweep,   
                          3 : DoubleLaneChange,
					      4 : CoastDown ,
					      5 : SingleTurn}
    experiment_sel 	= rospy.get_param("controller/experiment_sel")
    v_x_pwm 	= rospy.get_param("controller/v_x_pwm")
    t_0         = rospy.get_param("controller/t_0")
    t_exp 		= rospy.get_param("controller/t_exp")
    t_turn      = rospy.get_param("controller/t_turn")
    str_ang 	= rospy.get_param("controller/steering_angle")
    test_mode   = experiment_opt.get(experiment_sel)
    opt 	    = TestSettings(SPD = v_x_pwm, turn = str_ang, dt=t_exp)
    opt.t_turn = t_turn
    opt.t_0    = t_0

    activateLQR = False
    ignoreEncoder = 0

    # main loop
    while not rospy.is_shutdown():
        # check for LQR activation
        global x_hat

        # get current state estimate
        v_x 	= x_hat[0]
        v_y 	= x_hat[1]
        r 	    = x_hat[2]

	    # compute slip angle beta
        if v_x > 0:
            beta    = arctan(v_y / v_x)
        else:
            beta    = 0

        # compute error state
        z_hat       = array([beta, r, v_x]).reshape(-1,1) 
        z_error     = z_hat - z_eq
    
        # compute quadratic expression for LQR activation
        v_LQR   = dot(dot(z_error.T, P),z_error)[0][0]

        if v_LQR < gamma:
            activateLQR = True
            t0_LQR = time.time()

        # CLOSED LOOP run LQR if state meets activation condition
        if activateLQR:
            t = time.time()
            if v_x > 0:
                (motorCMD, servoCMD, d_f) = LQR_drift(z_eq, K_LQR, vhMdl, TrMdl, F_ext, u_eq, offsets)
                if v_x <= 2.0:
                    ignoreEncoder = 1
            else:
                (motorCMD, servoCMD, d_f) = (90,90,0)

            if t - t0_LQR > 10:
                motorCMD = 90

        # OPEN LOOP initially go straight and turn
        else:
            # get motor command
            (motorCMD, _) = test_mode(opt, rateHz, t_i)

            # get steering wheel command
            # go straight using PID
            if t_i < (rateHz*(t_0 + t_turn)):
                global err
                d_f_deg   = pid.update(err, dt)
                servoCMD  = angle_2_servo(d_f_deg)
            # after certain time, turn
            else:
                (_,servoCMD) = test_mode(opt, rateHz, t_i)
                d_f_deg     = servo_2_angle(servoCMD)
            d_f         = d_f_deg*pi/180

        # send command signal 
        ecu_cmd = Vector3(motorCMD, servoCMD, d_f)
        nh.publish(ecu_cmd)
        debug_msg   = array([err, d_f, v_LQR, ignoreEncoder], dtype = float32)
        debug.publish( debug_msg )
	
        # wait
        t_i += 1
        rate.sleep()

#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except rospy.ROSInterruptException:
		pass
