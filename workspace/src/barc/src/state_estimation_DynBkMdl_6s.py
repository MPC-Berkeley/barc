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
import time
import os
from barc.msg import ECU, Encoder, six_states
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from numpy import pi, cos, sin, eye, array, zeros
from observers import kinematicLuembergerObserver, ekf
from system_models import f_6s, h_6s
from tf import transformations
from numpy import unwrap

# input variables
d_f 	    = 0
FxR         = 0

# raw measurement variables from IMU
yaw_prev = 0
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)
yaw_local   = 0
read_yaw0   = False
psi         = 0
psi_meas    = 0


# raw measurement variables from indoor GPS
(X_gps, Y_gps) = zeros(2)

# raw measurement variables from optic flow

(v_x_optic,v_y_optic) = zeros(2)
# from encoder
v_x_enc 	= 0
t0 	        = time.time()
n_FL	    = 0                     # counts in the front left tire
n_FR 	    = 0                     # counts in the front right tire
n_FL_prev 	= 0
n_FR_prev 	= 0
r_tire 		= 0.04                  # radius from tire center to perimeter along magnets [m]
dx_qrt 	    = 2.0*pi*r_tire/4.0     # distance along quarter tire edge

# ecu command update
def ecu_callback(data):
    global FxR, d_f
    FxR         = data.motor        # input motor force [N]
    d_f         = data.servo        # input steering angle [rad]

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
    global yaw_prev, yaw0, read_yaw0, yaw_local, psi_meas

    # get orientation from quaternion data, and convert to roll, pitch, yaw
    # extract angular velocity and linear acceleration data
    ori  = data.orientation
    quaternion  = (ori.x, ori.y, ori.z, ori.w)
    (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)
    # save initial measurements
    if not read_yaw0:
        read_yaw0   = True
        yaw_prev    = yaw
        yaw0        = yaw
    
    # unwrap measurement
    yaw         = unwrap(array([yaw_prev, yaw]), discont = pi)[1]
    yaw_prev    = yaw
    yaw_local   = yaw - yaw0

    # extract angular velocity and linear acceleration data
    w_x = data.angular_velocity.x
    w_y = data.angular_velocity.y
    w_z = data.angular_velocity.z
    a_x = data.linear_acceleration.x
    a_y = data.linear_acceleration.y
    a_z = data.linear_acceleration.z
    #rospy.loginfo("I heard IMU")

# encoder measurement update
def enc_callback(data):
	global v_x_enc, d_f, t0
	global n_FL, n_FR, n_FL_prev, n_FR_prev

	n_FL = data.FL
	n_FR = data.FR

	# compute time elapsed
	tf = time.time()
	dt = tf - t0
	
	# if enough time elapse has elapsed, estimate v_x
	dt_min = 0.20
	if dt >= dt_min:
		# compute speed :  speed = distance / time
		v_FL = float(n_FL- n_FL_prev)*dx_qrt/dt
		v_FR = float(n_FR- n_FR_prev)*dx_qrt/dt

		# update encoder v_x, v_y measurements
		# only valid for small slip angles, still valid for drift?
		v_x_enc 	= (v_FL + v_FR)/2.0*cos(d_f)

		# update old data
		n_FL_prev   = n_FL
		n_FR_prev   = n_FR
		t0 	        = time.time()

# indoor gps measurement update
def gps_callback(data):
    global X_gps, Y_gps
    X_gps = data.x/100  #cm->m
    Y_gps = data.y/100  #cm->m
    #rospy.loginfo("I heard indoor gps")

# indoor gps measurement update
def optic_callback(data):
    global v_x_optic,v_y_optic
    v_x_optic = data.x
    v_y_optic = data.y
    #rospy.loginfo("I heard optic flow")

# state estimation node
def state_estimation():
	# initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.Subscriber('/encoder', Encoder, enc_callback)
    rospy.Subscriber('/ecu', ECU, ecu_callback)
    rospy.Subscriber('/indoor_gps',Vector3 , gps_callback)
    rospy.Subscriber('/vel_est',Vector3 , optic_callback)
    state_pub 	= rospy.Publisher('state_estimate', six_states, queue_size = 10)

	# get vehicle dimension parameters
    L_a = rospy.get_param("/L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("/L_b")       # distance from CoG to rear axel
    m   = rospy.get_param("/m")         # mass of vehicle
    I_z = rospy.get_param("/I_z")       # moment of inertia about z-axis
    vhMdl   = (L_a, L_b, m, I_z)

    # get encoder parameters
    dt_vx   = rospy.get_param("/state_estimation/dt_v_enc")     # time interval to compute v_x

    # get tire model
    B   = rospy.get_param("/tire_model/B")
    C   = rospy.get_param("/tire_model/C")
    mu  = rospy.get_param("/tire_model/mu")
    TrMdl = ([B,C,mu],[B,C,mu])

    # get external force model
    a0  = rospy.get_param("/air_drag_coeff")
    Ff  = rospy.get_param("/friction")

    # get EKF observer properties
    q_std   = rospy.get_param("/state_estimation/q_std")             # std of process noise
    r_std   = rospy.get_param("/state_estimation/r_std")             # std of measurementnoise
    v_x_min     = rospy.get_param("/state_estimation/v_x_min")  # minimum velociy before using EKF

	# set node rate
    loop_rate 	= 50
    dt 		    = 1.0 / loop_rate
    rate 		= rospy.Rate(loop_rate)
    t0 			= time.time()

    # estimation variables for Luemberger observer
    z_EKF       = array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])

    # estimation variables for EKF
    P           = eye(6)                # initial dynamics coveriance matrix
    R           = array([[0.01, 0, 0, 0, 0, 0],
                         [0, 0.01, 0, 0, 0, 0],
                         [0, 0, 0.01, 0, 0, 0],
                         [0, 0, 0, 100.0, 0, 0],
                         [0, 0, 0, 0, 100.0, 0],
                         [0, 0, 0, 0, 0, 1.0]])   # measurement noise coveriance matrix
  
    Q           = array([[0.01, 0, 0, 0, 0, 0],
                         [0, 0.01, 0, 0, 0, 0],
                         [0, 0, 0.01, 0, 0, 0],
                         [0, 0, 0, 10.0, 0, 0],
                         [0, 0, 0, 0, 10.0, 0],
                         [0, 0, 0, 0, 0, 10.0]])    # process noise coveriance matrix  
    #time(&time_now);
    #struct tm*now = localtime(&time_now);
    #std::stringstream ss_mon,ss_day,ss_hour,ss_min,ss_sec;
    #ss_mon << now->tm_mon+1;
    #ss_day << now->tm_mday;
    #ss_hour << now->tm_hour;
    #ss_min << now->tm_min;
    #ss_sec << now->tm_sec;
    #std::string s_mon = ss_mon.str();
    #std::string s_day = ss_day.str();
    #std::string s_hour = ss_hour.str();
    #std::string s_min = ss_min.str();
    #std::string s_sec = ss_sec.str();
    #std::ofstream Data (("/home/odroid/Data/Drift_corner/"+s_mon+ "."+s_day+"."+s_hour+"."+s_min+"."+s_sec +".csv").c_str());
    #Data << "t,X_raw,Y_raw,yaw_raw,vx_raw,vy_raw,yawrate_raw,X,Y,yaw,vx,vy,yawrate,steeing,F_xR"<<std::endl;
    date 				= time.strftime("%Y.%m.%d")
    BASE_PATH   		= "/home/odroid/Data/" + date + "/"
    if not os.path.exists(BASE_PATH):
        os.makedirs(BASE_PATH)
    data_file_name   	= BASE_PATH + time.strftime("%H.%M.%S") + '.csv'
    data_file     		= open(data_file_name, 'a')
    columns             = 't,' + \
                          'X_raw,Y_raw,yaw_raw,' +  \
                          'vx_raw,vy_raw,r_raw,' + \
                          'X,Y,yaw,vx,vy,r,' + \
                          'd_f,FxR\n'
    data_file.write(columns)
    while not rospy.is_shutdown():

		# publish state estimate
        (X, Y, phi, v_x, v_y, r) = z_EKF           # note, r = EKF estimate yaw rate

        # publish information
        state_pub.publish( six_states(X, Y, phi, v_x, v_y, r) )
        # Data << t << "," << X_gps << "," << Y_gps << "," << yaw << "," << v_x_optic << "," << v_y_optic << "," << w_z<< X << "," << Y << "," << phi << "," << vx << "," << vy << "," << yr << "," << d_f << "," << F_xR << "," <<std::endl;
        t  	= time.time() - t0
        raw_data    = [X_gps,Y_gps,yaw_local,v_x_optic,v_y_optic,w_z]
        filter_data    = [X,Y,phi,v_x,v_y,r]
        command = [d_f,FxR]
        all_data    = [t] + raw_data + filter_data + command 
        N = len(all_data)
        str_fmt = '%.4f,'*N
        data_file.write( (str_fmt[0:-1]+'\n') % tuple(all_data))

        # apply EKF
        if v_x_optic > v_x_min:
            # get measurement
            y = array([X_gps, Y_gps, yaw_local, v_x_optic, v_y_optic, w_z])

            # define input
            u       = array([ d_f, FxR ])

            # build extra arguments for non-linear function
            F_ext = array([ a0, Ff ]) 
            args = (u, vhMdl, TrMdl, F_ext, dt) 

            # apply EKF and get each state estimate
            (z_EKF,P) = ekf(f_6s, z_EKF, P, h_6s, y, Q, R, args )

        else:
            z_EKF[0] = float(X_gps)
            z_EKF[1] = float(Y_gps)
            z_EKF[2] = float(yaw_local)
            z_EKF[3] = float(v_x_optic)
            z_EKF[4] = float(v_y_optic)
            z_EKF[5] = float(w_z)

		# wait
        rate.sleep()
        rospy.loginfo("yaw angle : %f",yaw_local)

if __name__ == '__main__':
	try:
	   state_estimation()
	except rospy.ROSInterruptException:
		pass
