#!/usr/bin/env julia

#=
	File name: estimator.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#

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

using RobotOS
using Polynomials
@rosimport barc.msg: ECU, pos_info, Vel_est, prediction
@rosimport sensor_msgs.msg : Imu
@rosimport marvelmind_nav.msg : hedge_pos
@rosimport std_msgs.msg : Header
rostypegen()

using barc.msg
using sensor_msgs.msg
using marvelmind_nav.msg
using std_msgs.msg

include("config_2.jl")
include("track.jl")
include("transformations.jl")

#=
from numpy import eye, array, zeros, diag, unwrap, tan, cos, sin, vstack, linalg, append
from numpy import ones, polyval, delete, size
from observers import ekf
from system_models import f_SensorKinematicModel, h_SensorKinematicModel
from tf import transformations
import math
=#

# ***_meas are values that are used by the Kalman filters
# ***_raw are raw values coming from the sensors


function quaternion_to_euler(quaternion::Array{Float64})
	q_x, q_y, q_z, q_w = quaternion

	# Roll: X-axis roation
	sin_roll = 2.0 * (q_w * q_x + q_y * q_z)
	cos_roll = 1.0 - 2.0 * (q_x^2 + q_y^2)
	roll = atan2(sin_roll, cos_roll)

	# Pitch: Y-axis rotation
	sin_pitch = 2.0 * (q_w * q_y - q_z * q_x)
	if abs(sin_pitch) >= 1.0
		pitch = copysign(pi / 2, sin_pitch)
	else
		pitch = asin(sin_pitch)
	end

	# Yaw: Z-axis rotation
	sin_yaw = 2.0 * (q_w * q_z + q_x * q_y)
	cos_yaw = 1.0 - 2.0 * (q_y^2 + q_z^2)
	yaw = atan2(sin_yaw, cos_yaw)

	return [roll, pitch, yaw]
end


function unwrap(v, inplace=false)
	# From: https://gist.github.com/ssfrr/7995008
	# currently assuming an array
    unwrapped = inplace ? v : copy(v)
    for i in 2:length(v)
        while unwrapped[i] - unwrapped[i - 1] >= pi
        	unwrapped[i] -= 2 * pi
    	end
    	while unwrapped[i] - unwrapped[i - 1] <= - pi
      		unwrapped[i] += 2 * pi
    	end
  	end

  	return unwrapped
end

unwrap!(v) = unwrap(v, true)


type StateEstimator
	cmd_servo::Float64
    cmd_motor::Float64
    cmd_t::Float64

    # IMU
    imu_updated::Bool
    yaw_prev::Float64
    yaw0::Float64  # yaw at t = 0
    yaw_meas::Float64
    psiDot_meas::Float64
    a_x_meas::Float64
    a_y_meas::Float64
    att::Array{Float64}  # attitude

    # Velocity
    vel_updated::Bool
    vel_meas::Float64
    vel_prev::Float64
    vel_count::Float64 # counts how often the same vel meas. has been received

    # GPS
    gps_updated::Bool
    x_meas::Float64
    y_meas::Float64
    x_hist::Array{Float64}
    y_hist::Array{Float64}
    t_gps::Array{Float64}
    c_X::Polynomials.Poly{Float64}
    c_Y::Polynomials.Poly{Float64}

    # Estimator data
    x_est::Float64
    y_est::Float64

    # General variables
    running::Bool  # bool if the car is driving

   	t0::Float64

    z_EKF::Array{Float64}  # x, y, psi, v, psi_drift
    P::Array{Float64}  # initial dynamics coveriance matrix

    qa::Float64
    qp::Float64

    Q::Array{Float64}
    R::Array{Float64}

    d_f_hist::Array{Float64}  # assuming that we are running at 50Hz, array of 10 means 0.2s lag
    d_f_lp::Float64
    a_lp::Float64

    t_now::Float64

    s_coord::Array{Float64}
    u::Array{Float64}
    dt::Float64

    counter::Int64

    sub_imu::RobotOS.Subscriber{sensor_msgs.msg.Imu}
    sub_vel::RobotOS.Subscriber{barc.msg.Vel_est}
    sub_ecu::RobotOS.Subscriber{barc.msg.ECU}
    sub_gps::RobotOS.Subscriber{marvelmind_nav.msg.hedge_pos}

    pub_xy::RobotOS.Publisher{barc.msg.pos_info}
    xy_state::pos_info
    xy_coord::Array{Float64}

    StateEstimator() = new()
end

function init!(estimator::StateEstimator, dt::Float64)
	# input variables
    estimator.cmd_servo = 0.0
    estimator.cmd_motor = 0.0
    estimator.cmd_t = 0.0

    # IMU
    estimator.yaw_prev = 0.0
    estimator.yaw0 = 0.0            # yaw at t = 0
    estimator.yaw_meas = 0.0
    estimator.psiDot_meas = 0.0
    estimator.a_x_meas = 0.0
    estimator.a_y_meas = 0.0
    estimator.imu_updated = false
    estimator.att = zeros(3)               # attitude

    # Velocity
    estimator.vel_meas = 0.0
    estimator.vel_updated = false
    estimator.vel_prev = 0.0
    estimator.vel_count = 0.0 

    # GPS
    estimator.x_meas = 0.0
    estimator.y_meas = 0.0
    estimator.gps_updated = false
    estimator.x_hist = zeros(15)
    estimator.y_hist = zeros(15)
    estimator.t_gps = zeros(15)

    # Estimator data
    estimator.x_est = 0.0
    estimator.y_est = 0.0

    # General variables
    estimator.running = false

    t0 = get_rostime()
    estimator.t0 = to_sec(t0) # set initial time

    estimator.z_EKF = zeros(14)  # x, y, psi, v, psi_drift
    estimator.P = eye(14)  # initial dynamics coveriance matrix

    estimator.qa = 1000.0
    estimator.qp = 1000.0

    #         x, y, vx, vy, ax, ay, psi, psidot, psidrift, x, y, psi, v
    estimator.Q = diagm([1 / 20 * dt^5 * estimator.qa, 1 / 20 * dt^5 * estimator.qa, 
    			  1 / 3 * dt^3 * estimator.qa, 1 / 3 * dt^3 * estimator.qa, 
    			  dt * estimator.qa, dt * estimator.qa, 1 / 3 * dt^3 * estimator.qp, 
    		   	  dt * estimator.qp, 0.1, 0.2, 0.2, 1.0, 1.0, 0.1])
    estimator.R = diagm([5.0, 5.0, 1.0, 10.0, 100.0, 1000.0, 1000.0, 5.0, 5.0, 10.0, 1.0, 
    		   			 10.0, 10.0])
    #R = diag([20.0,20.0,1.0,10.0,100.0,1000.0,1000.0,     20.0,20.0,10.0,1.0, 10.0,10.0])
    #         x,y,v,psi,psiDot,a_x,a_y, x, y, psi, v

    estimator.d_f_hist = zeros(10)  # assuming that we are running at 50Hz, array of 10 means 0.2s lag
    estimator.d_f_lp = 0.0
    estimator.a_lp = 0.0

    estimator.t_now = 0.0

    estimator.s_coord = zeros(6)
    estimator.u = zeros(2)
    estimator.dt = dt

    estimator.counter = 0

    # topic subscriptions
    estimator.sub_imu = Subscriber("imu/data", Imu, imu_callback, (estimator,), 
    							   queue_size=10)::RobotOS.Subscriber{sensor_msgs.msg.Imu}
    estimator.sub_vel = Subscriber("vel_est", Vel_est, encoder_vel_callback, (estimator,), 
    							   queue_size=10)::RobotOS.Subscriber{barc.msg.Vel_est}
    estimator.sub_ecu = Subscriber("ecu", ECU, ecu_callback, (estimator,), 
    							   queue_size=10)::RobotOS.Subscriber{barc.msg.ECU}
    estimator.sub_gps = Subscriber("hedge_pos", hedge_pos, gps_callback, (estimator,), 
    							   queue_size=10)::RobotOS.Subscriber{marvelmind_nav.msg.hedge_pos}
    estimator.pub_xy = Publisher("pos_info", pos_info, 
    							 queue_size=1)::RobotOS.Publisher{barc.msg.pos_info}
    estimator.xy_state = pos_info()
    estimator.xy_coord = zeros(6)
end


function ecu_callback(msg::ECU, estimator::StateEstimator)
	estimator.cmd_motor = msg.motor  # input motor force in Newton
	estimator.cmd_servo = msg.servo  # input steering angle in radian
	# Set running to true once the first command is received.
	# Here the yaw is going to be set to zero. 
	if !estimator.running
		estimator.running = true
	end

	# define input
    # this is for a 0.2 seconds delay of steering
    append!(estimator.d_f_hist, [estimator.cmd_servo])  
    # low pass filter on steering
    estimator.d_f_lp = estimator.d_f_lp + 0.5 * (estimator.cmd_servo - estimator.d_f_lp) 
    # low pass filter on acceleration
    estimator.a_lp = estimator.a_lp + 1.0 * (estimator.cmd_motor - estimator.a_lp)       

    estimator.u = [estimator.cmd_motor, estimator.d_f_hist[1]]
    deleteat!(estimator.d_f_hist, 1)
end

function gps_callback(msg::hedge_pos, estimator::StateEstimator)
	# This function is called when a new GPS signal is received.
    # units: [rad] and [rad/s]
    # get current time stamp
    # TODO: Don't know if this will work
    t_now = to_sec(get_rostime()) - estimator.t0

    # get current gps measurement 
    estimator.x_meas = msg.x_m
    estimator.y_meas = msg.y_m

    # check if we have good measurement
    # compute distance we have travelled from previous estimate to current 
    # measurement, if we've travelled more than 1 m, the GPS measurement is 
    # probably junk, so ignore it otherwise, store measurement, and then 
    # perform interpolation.
    dist = (estimator.x_est - msg.x_m)^2 + 
    	   (estimator.y_est - msg.y_m)^2

    if dist < 1.0
        append!(estimator.x_hist, [msg.x_m])
        append!(estimator.y_hist, [msg.y_m])
        append!(estimator.t_gps, [t_now])
    end

    # Keep only the last second worth of coordinate data in the x_hist and 
    # y_hist buffer. These buffers are used for interpolation, without 
    # overwriting old data, the arrays would grow unbounded
    # TODO: Don't know if this will work
    estimator.x_hist = estimator.x_hist[estimator.t_gps .> t_now - 1.0]
    estimator.y_hist = estimator.y_hist[estimator.t_gps .> t_now - 1.0]
    estimator.t_gps = estimator.t_gps[estimator.t_gps .> t_now - 1.0]
    println("t_gps: ", estimator.t_gps)
    sz = size(estimator.t_gps, 1)

    # perform interpolation for (x,y) as a function of time t
    # getting two function approximations x(t) and y(t)
    # 1) x(t) ~ c0x + c1x * t + c2x * t^2
    # 2) y(t) ~ c0y + c1y * t + c2y * t^2
    # c_X = [c0x c1x c2x] and c_Y = [c0y c1y c2y] 
    # use least squares to get the coefficients for this function approximation 
    # using (x,y) coordinate data from the past second (time)
    if sz >= 4
        estimator.c_X = polyfit(estimator.t_gps, estimator.x_hist, 3)
        estimator.c_Y = polyfit(estimator.t_gps, estimator.y_hist, 3)
    end

    estimator.gps_updated = true
end
    
function imu_callback(msg::Imu, estimator::StateEstimator)
    # units: [rad] and [rad/s]
    current_t = to_sec(get_rostime())

    # get orientation from quaternion data, and convert to roll, pitch, yaw
    # extract angular velocity and linear acceleration data
    ori = msg.orientation
    quaternion = [ori.x, ori.y, ori.z, ori.w]
    roll_raw, pitch_raw, yaw_raw = quaternion_to_euler(quaternion)
    # yaw_meas is element of [-pi,pi]

    yaw = unwrap([estimator.yaw_prev, yaw_raw])[2]	 # get smooth yaw (from beginning)
    estimator.yaw_prev = estimator.yaw_meas  # and always use raw measured yaw for unwrapping
    # from this point on 'yaw' will be definitely unwrapped (smooth)!
    if !estimator.running
        estimator.yaw0 = yaw  # set yaw0 to current yaw
        estimator.yaw_meas = 0.0  # and current yaw to zero
    else
        estimator.yaw_meas = yaw - estimator.yaw0
        #print "yaw measured: %f" % self.yaw_meas * 180 / math.pi
    end

    # extract angular velocity and linear acceleration data
    #w_x = data.angular_velocity.x
    #w_y = data.angular_velocity.y
    w_z = msg.angular_velocity.z
    a_x = msg.linear_acceleration.x
    a_y = msg.linear_acceleration.y
    a_z = msg.linear_acceleration.z

    estimator.psiDot_meas = w_z
    # The next two lines 'project' the measured linear accelerations to a horizontal plane
    estimator.a_x_meas = cos(- pitch_raw) * a_x + sin(- pitch_raw) * sin(- roll_raw)* a_y - 
    					 sin(- pitch_raw) * cos(- roll_raw) * a_z
    estimator.a_y_meas = cos(- roll_raw) * a_y + sin(- roll_raw) * a_z
    estimator.att = [roll_raw, pitch_raw, yaw_raw]
    estimator.imu_updated = true
end
        
function encoder_vel_callback(msg::Vel_est, estimator::StateEstimator)
    if msg.vel_est != estimator.vel_prev
        estimator.vel_meas = msg.vel_est
        estimator.vel_updated = true
        estimator.vel_prev = msg.vel_est
        estimator.vel_count = 0
    else
        estimator.vel_count = estimator.vel_count + 1
        if estimator.vel_count > 10  # if 10 times in a row the same measurement
            estimator.vel_meas = 0  # set velocity measurement to zero
            estimator.vel_updated = true
        end
    end
end


function extended_kalman_filter(estimator::StateEstimator, f, h, y_kp1, args)
    #=
     EKF   Extended Kalman Filter for nonlinear dynamic systems
     ekf(f,mx,P,h,z,Q,R) returns state estimate, x and state covariance, P 
     for nonlinear dynamic system:
               x_k+1 = f(x_k) + w_k
               y_k   = h(x_k) + v_k
     where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
           v ~ N(0,R) meaning v is gaussian noise with covariance R
    Inputs:    f: function handle for f(x)
               mx_k: "a priori" state estimate
               P_k: "a priori" estimated state covariance
               h: fanction handle for h(x)
               y_kp1: current measurement
               Q: process noise covariance 
               R: measurement noise covariance
               args: additional arguments to f(x, *args)
    Output:    mx_kp1: "a posteriori" state estimate
               P_kp1: "a posteriori" state covariance
               
    Notation: mx_k = E[x_k] and my_k = E[y_k], where m stands for "mean of"
    =#
    
    # f_SensorKinematicModel, P, h_SensorKinematicModel, y, args
    # args = [estimator.u, vhMdl, estimator.dt]

    xDim    = size(estimator.z_EKF)[1]  # dimension of the state
    mx_kp1  = f(estimator.z_EKF, args)  # predict next state
    A       = numerical_jac(f, estimator.z_EKF, args)  # linearize process model about current state
    P_kp1   = (A * estimator.P_k) * transpose(A) + estimator.Q  # proprogate variance
    my_kp1  = h(mx_kp1, args)  # predict future output
    H       = numerical_jac(h, mx_kp1)  # linearize measurement model about predicted next state
    P12     = P_kp1 * transpose(H)  # cross covariance
    K       = P12 * inv(H * P12 + estimator.R)  # Kalman filter gain
    mx_kp1  = mx_kp1 + K * (y_kp1 - my_kp1)  # state estimate
    P_kp1   = K * estimator.R * transpose(K) + 
    		  (eye(xDim) - K * H) * P_kp1 * transpose(eye(xDim) - K * H)

    return [mx_kp1, P_kp1]
end
    
function numerical_jac(f, x, args...)
    #=
    Function to compute the numerical jacobian of a vector valued function 
    using final differences
    =#

    # numerical gradient and diagonal hessian
    # args = (estimator.u, vhMdl, estimator.dt)
    y = f(x, args)
    
    jac = zeros(size(y, 1), size(x, 1))
    eps = 1e-5
    xp = copy(x)
    
    for i = 1 : size(x, 1)
        xp[i] = x[i] + eps / 2.0
        yhi = f(xp, args)
        xp[i] = x[i] - eps / 2.0
        ylo = f(xp, args)
        xp[i] = x[i]
        jac[:, i] = (yhi - ylo) / eps
    end

    return jac
end

function f_SensorKinematicModel(z, args)
    # This Sensor model contains a pure Sensor-Model and a Kinematic model. 
    # They're independent from each other.
    u = zeros(2)
    u[1], u[2], l_front, l_rear, dt = args
    beta = atan2(l_front * tan(u[2]), l_front + l_rear)
    zNext = zeros(14)
    zNext[1] = z[1] + dt * (cos(z[7]) * z[3] - sin(z[7]) * z[4])    # x
    zNext[2] = z[2] + dt * (sin(z[7]) * z[3] + cos(z[7]) * z[4])    # y
    zNext[3] = z[3] + dt * (z[5] + z[8] * z[4])                     # v_x
    zNext[4] = z[4] + dt * (z[6] - z[8] * z[3])                     # v_y
    zNext[5] = z[5]                                                 # a_x
    zNext[6] = z[6]                                                 # a_y
    zNext[7] = z[7] + dt * z[8]                                     # psi
    zNext[8] = z[8]                                                 # psidot
    zNext[9] = z[9]                                                 # drift_psi
    zNext[10] = z[10] + dt * (z[13] * cos(z[12] + beta))             # x
    zNext[11] = z[11] + dt * (z[13] * sin(z[12] + beta))            # y
    zNext[12] = z[12] + dt * (z[13] / l_rear * sin(beta))           # psi
    zNext[13] = z[13] + dt * (u[1] - 0.5 * z[13])                   # v
    zNext[14] = z[14]                                               # drift_psi_2
    return zNext
end

function h_SensorKinematicModel(x, args...)
    # This is the measurement model to the kinematic<->sensor model above
    y = zeros(13)
    y[1] = x[1]                     # x
    y[2] = x[2]                     # y
    y[3] = sqrt(x[3]^2 + x[4]^2)    # v
    y[4] = x[7] + x[9]              # psi
    y[5] = x[8]                     # psiDot
    y[6] = x[5]                     # a_x
    y[7] = x[6]                     # a_y
    y[8] = x[10]                    # x
    y[9] = x[11]                    # y
    y[10] = x[12] + x[14]           # psi
    y[11] = x[13]                   # v
    y[12] = x[3]                    # v_x
    y[13] = x[4]                    # v_y
    return y
end

function estimate(estimator::StateEstimator, track::Track, vhMdl::Array{Float64})
    estimator.t_now = to_sec(get_rostime()) - estimator.t0

    estimator.x_meas = estimator.c_X(estimator.t_now)
    estimator.y_meas = estimator.c_Y(estimator.t_now)
    estimator.gps_updated = false
    estimator.imu_updated = false
    estimator.vel_updated = false

    bta = 0.5 * estimator.u[2]

    # print "V, V_x and V_y : (%f, %f, %f)" % (se.vel_meas,cos(bta)*se.vel_meas, sin(bta)*se.vel_meas)

    # get measurement
    y = [estimator.x_meas, estimator.y_meas, estimator.vel_meas, 
    	 estimator.yaw_meas, estimator.psiDot_meas, estimator.a_x_meas, 
    	 estimator.a_y_meas, estimator.x_meas, estimator.y_meas, 
    	 estimator.yaw_meas, estimator.vel_meas, cos(bta) * estimator.vel_meas, 
    	 sin(bta) * estimator.vel_meas]

    # build extra arguments for non-linear function
    args = [estimator.u; vhMdl; estimator.dt]
    println(args)

    # apply EKF and get each state estimate
    estimator.z_EKF, estimator.P = extended_kalman_filter(estimator, 
														  f_SensorKinematicModel, 
														  h_SensorKinematicModel, 
														  y, args)

    # Read values
    x_est, y_est, v_x_est, v_y_est, 
    a_x_est, a_y_est, psi_est, psi_dot_est, 
    psi_drift_est, x_est_2, y_est_2, 
    psi_est_2, v_est_2, psi_drift_est_2 = estimator.z_EKF           # note, r = EKF estimate yaw rate

    estimator.x_est = x_est_2
    estimator.y_est = y_est_2
    #print "V_x and V_y : (%f, %f)" % (v_x_est, v_y_est)

    # Update track position
    estimator.xy_coord = [x_est_2, y_est_2, v_x_est, v_y_est, psi_est_2, psi_dot_est]

    # Calculate new s, ey, epsi (only 12.5 Hz, enough for controller that runs at 10 Hz)
    # TODO: don't need this anymore, because we're doing this right before 
    # the controller
    # if estimator.counter % 4 == 0:
    #     estimator.s_coord = xy_to_s(track, xy_coord)

    #=
    estimator.s_coord = xy_to_s(track, xy_coord)
    [s, e_y, e_psi, psi_dot, v_x, v_y] = estimator.s_coord
    [x, y, v_x, v_y, psi, psi_dot] = xy_coord
    =#

    #=
    # and then publish position info
    ros_t = rospy.get_rostime()
    state_pub_pos.publish(pos_info(Header(stamp=ros_t), l.s, l.ey, l.epsi, v_est_2, l.s_start, l.x, l.y, l.v_x, l.v_y,
                                   l.psi, l.psiDot, se.x_meas, se.y_meas, se.yaw_meas, se.vel_meas, se.psiDot_meas,
                                   psi_drift_est, a_x_est, a_y_est, se.a_x_meas, se.a_y_meas, se.cmd_motor, se.cmd_servo,
                                   (0,), (0,), (0,), l.coeffCurvature.tolist()))
	=#

    # wait
    estimator.counter += 1

	return estimator.xy_coord    
end

function publish_current_xy(estimator::StateEstimator, xy_coords::Array{Float64})
	estimator.xy_state.header.stamp = to_sec(get_rostime())
	estimator.xy_state.x = xy_coords[1]
	estimator.xy_state.y = xy_coords[2]
	estimator.xy_state.v_x = xy_coords[3]
	estimator.xy_state.v_y = xy_coords[4]
	estimator.xy_state.psi = xy_coords[5]
	estimator.xy_state.psiDot = xy_coords[6]
	publish(estimator.pub_xy, estimator.xy_state)
end

if !isinteractive()
	init_node("estimator_node")

	l_front = get_param("L_a")
    l_rear = get_param("L_b")

    vhMdl = [l_front, l_rear]

    loop_rate = 50
    dt = 1.0 / loop_rate
    rate = Rate(loop_rate)

	track = Track()
    init!(track)

    estimator = StateEstimator()
    init!(estimator, dt)

	while !is_shutdown()
		try
	    	current_xy = estimate(estimator, track, vhMdl)
	    	publish_current_xy(estimator, current_xy)
	   	catch y
	   		if isa(y, UndefRefError)
	    		println("Estimation failed")
	    	else
	    		current_xy = estimate(estimator, track, vhMdl)
	    		publish_current_xy(estimator, current_xy)
	    	end
	   	end

    	rossleep(rate)
    end
end

#=
if __name__ == '__main__':
    try:
        state_estimation()
    except rospy.ROSInterruptException:
        pass

=#
