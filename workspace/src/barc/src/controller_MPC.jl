#!/usr/bin/env julia

#=
 Licensing Information: You are free to use or extend these projects for
 education or reserach purposes provided that (1) you retain this notice
 and (2) you provide clear attribution to UC Berkeley, including a link
 to http://barc-project.com

 Attibution Information: The barc project ROS code-base was developed
 at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
 (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
 by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
 based on an open source project by Bruce Wootton
=#

using RobotOS
@rosimport barc.msg: ECU, Encoder, Ultrasound, Z_KinBkMdl
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using JuMP
using Ipopt
using DataFrames

# Open and prepare throttle map
throttle_map = readtable("/home/odroid/throttlemap1.csv", header=false)
pwm_range = convert(Array{Int32}, throttle_map[1])
accel_range = convert(Array{Float32}, throttle_map[2])

# define model parameters
L_a     = 0.125         # distance from CoG to front axel
L_b     = 0.125         # distance from CoG to rear axel
dt      = 0.1           # time step of system

# define model constraints
a_max   = 1.5
a_min   = -1.3
v_max = 2
v_min = -1*v_max
d_f_max = 30*pi/180.0
d_f_min = -1*d_f_max

# preview horizon
N       = 5

# define targets (meters)
x_ref   = 2
y_ref   = 0

# define decision variables
# states: position (x,y), yaw angle, and velocity
# inputs: acceleration, steering angle
println("Creating kinematic bicycle model ....")
mdl     = Model(solver = IpoptSolver(print_level=3))
@defVar( mdl, x[1:(N+1)] )
@defVar( mdl, y[1:(N+1)] )
@defVar( mdl, psi[1:(N+1)] )
@defVar( mdl, v_min <= v[1:(N+1)] <= v_max)
@defVar( mdl, d_f_min <= d_f[1:N] <= d_f_max)

# TODO use actuator rate limiting constraints if needed

# define objective function
# @setNLObjective(mdl, Min, (x[N+1] - x_ref)^2 + (y[N+1] - y_ref)^2) overshoots
# because no velocity penalty
@setNLObjective(mdl, Min, (x[N+1] - x_ref)^2 + (y[N+1] - y_ref)^2 + (v[N+1])^2)

# define constraints
# define system dynamics
# Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series,
#               Spring, 2011, page 26
@defNLParam(mdl, x0     == 0); @addNLConstraint(mdl, x[1]     == x0);
@defNLParam(mdl, y0     == 0); @addNLConstraint(mdl, y[1]     == y0);
@defNLParam(mdl, psi0   == 0); @addNLConstraint(mdl, psi[1]   == psi0 );
@defNLParam(mdl, v0     == 0); @addNLConstraint(mdl, v[1]     == v0);
@defNLExpr(mdl, bta[i = 1:N], atan( L_b / (L_a + L_b) * tan(d_f[i]) ) )
for i in 1:N
    @addNLConstraint(mdl, x[i+1]    == x[i]      + dt*(v[i]*cos( psi[i] + bta[i] ))  )
    @addNLConstraint(mdl, y[i+1]    == y[i]      + dt*(v[i]*sin( psi[i] + bta[i] ))  )
    @addNLConstraint(mdl, psi[i+1]  == psi[i]    + dt*(v[i]/L_b * sin(bta[i]))  )
    @addNLConstraint(mdl, v[i+1]    == v[i]      + dt*(a[i])  )
end

# status update
println("initial solve ...")
solve(mdl)
println("finished initial solve!")

function SE_callback(msg::Z_KinBkMdl)
    # update mpc initial condition
    setValue(x0,    msg.x)
    setValue(y0,    msg.y)
    setValue(psi0,  msg.psi)
    setValue(v0,    msg.v)
end

# Degrees to servo pwm command
function angle_2_servo(x)
    x = x-2
    u = 92.0558 + 1.8194*x  - 0.0104*x^2
    return u
end

# Accel m/s^2 to esc pwm command
function accel_2_pwm(a)
    pwm = nearest_pwm(a)
    if a > 0 # may want to give this a little epsilon around zero?
        pwm = max(95, pwm)
    else # a < 0
        pwm = min(87, pwm)
    end
    return pwm
end

# Find best ESC PWM for desired acceleration
function nearest_pwm(a_des)
    best_idx = 0
    min_err = 10
    for i=[1:length(accel_range)]
        err = abs(accel_range[i] - a_des)
        if err < min_err
            best_idx = i
            min_err = err
        end
    end
    return pwm_range[best_idx]
end

function main()
    # initiate node, set up publisher / subscriber topics
    init_node("mpc")
    pub = Publisher("ecu", ECU, queue_size=10)
    s1  = Subscriber("state_estimate", Z_KinBkMdl, SE_callback, queue_size=10)
    loop_rate = Rate(10)

    while ! is_shutdown()
        # run mpc, publish command
        solve(mdl)

        # get optimal solutions
        a_opt   = getValue(a[1])
        d_f_opt = getValue(d_f[1])

        # publish commands
        esc_cmd = accel_2_pwm(a_opt)
        # TODO verify that these conversions produce sensible output
        servo_cmd = angle_2_servo(d_f_opt*180/pi)
        cmd = ECU(esc_cmd, servo_cmd)
        publish(pub, cmd)

        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
