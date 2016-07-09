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
@rosimport barc.msg: ECU, Encoder, Ultrasound, Z_KinBkMdl, vel_sgn
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
#@rosimport std_msgs.msg: Bool
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
#using std_msgs.msg
using JuMP
using Ipopt
using DataFrames

# throttle map
throttle_map = readtable("/home/odroid/barc/workspace/src/barc/src/throttlemap1.csv")
pwm_range = convert(Array{Int32}, throttle_map[1])
accel_range = convert(Array{Float32}, throttle_map[2])

# define model parameters
L_a     = 0.125         # distance from CoG to front axle
L_b     = 0.125         # distance from CoG to rear axle
df      = 0.07          # distance from front axis to front
dr      = 0.06          # distance from back axis to back
Lf      = L_a + df      
Lr      = L_b + dr
w       = 0.095         # half the actual width
width   = w*2
len     = Lf + Lr;
dt      = 0.2           # time step of system

# preview horizon
N       = 7

# define targets [generic values] and updated in SE_callback based on initial measured psi_ref
# x_ref will initially be x_int
# after reaching x_int, x_ref will be set to x_final
x_ref   = 0
y_ref   = 0
psi_ref = 0
v_ref   = 0
x_final     = 3
y_final     = -0.6
psi_final   = 0
v_final     = 0
x_int       = x_final + 0.4
y_int       = y_final + 0.225
psi_int     = 0
v_int       = 0

# Parking spot definition, will be updated in set_target to relative coordinates
xl = x_final - (Lr + 0.07)
xr = x_final + Lf + 0.1
yt = y_final + w
spot_l = xr - xl
spot_w = width + 0.06

# Need to set psi_ref from relative initial yaw
read_yaw0 = 0
psi_offset = 0

# some model constraints
a_max = 1.5
a_min = -1.3
v_max = 2
v_min = -1*v_max
d_f_max = 30*pi/180.0
d_f_min = -1*d_f_max

x_min = -20
x_max = 20
y_min = -5
y_max = 20
psi_min = -4*pi
psi_max = 4*pi

# define decision variables 
# states: position (x,y), yaw angle, and velocity
# inputs: acceleration, steering angle 
println("Creating kinematic bicycle model ....")
mdl     = Model(solver = IpoptSolver(print_level=3))
@defVar( mdl, x_min <= x[1:(N+1)] <= x_max )
@defVar( mdl, y_min <= y[1:(N+1)] <= y_max )
@defVar( mdl, psi_min <= psi[1:(N+1)] <= psi_max )
@defVar( mdl, v_min <= v[1:(N+1)] <= v_max)
@defVar( mdl, a_min <= a[1:N] <= a_max)
@defVar( mdl, d_f_min <= d_f[1:N] <= d_f_max)
@defVar( mdl, d[1:16*N] )

# define objective function
@setNLObjective(mdl, Min, (x[N+1] - x_ref)^2 + (y[N+1] - y_ref)^2 + (psi[N+1] - psi_ref)^2 + (v[N+1] - v_ref)^2)

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
    @addNLConstraint(mdl, y[i+1]    == y[i]      + dt*(v[i]*sin( psi[i] + bta[i] ))  )
    @addNLConstraint(mdl, psi[i+1]  == psi[i]    + dt*(v[i]/L_b * sin(bta[i]))  )
    @addNLConstraint(mdl, v[i+1]    == v[i]      + dt*(a[i])  )

    @addNLConstraint(mdl, d[(i-1)*16 + 1]*(x[i] + Lf*cos(psi[i]) - width/2*sin(psi[i]) - xl ) 
                   + d[(i-1)*16 + 2]*(y[i] + Lf*sin(psi[i]) + width/2*cos(psi[i]) - yt) >= 0)
    @addNLConstraint(mdl, d[(i-1)*16 + 1] + d[(i-1)*16 + 2] == 1)               

    @addNLConstraint(mdl, d[(i-1)*16 + 3]*(x[i] + Lf*cos(psi[i]) + width/2*sin(psi[i]) - xl ) 
                   + d[(i-1)*16 + 4]*(y[i] + Lf*sin(psi[i]) - width/2*cos(psi[i]) - yt) >= 0)
    @addNLConstraint(mdl, d[(i-1)*16 + 3] + d[(i-1)*16 + 4] == 1)               

    @addNLConstraint(mdl, d[(i-1)*16 + 5]*(x[i] - Lr*cos(psi[i]) + width/2*sin(psi[i]) - xl ) 
                   + d[(i-1)*16 + 6]*(y[i] - Lr*sin(psi[i]) - width/2*cos(psi[i]) - yt) >= 0)
    @addNLConstraint(mdl, d[(i-1)*16 + 5] + d[(i-1)*16 + 6] == 1)               

    @addNLConstraint(mdl, d[(i-1)*16 + 7]*(x[i] - Lr*cos(psi[i]) - width/2*sin(psi[i]) - xl ) 
                   + d[(i-1)*16 + 8]*(y[i] - Lr*sin(psi[i]) + width/2*cos(psi[i]) - yt) >= 0)
    @addNLConstraint(mdl, d[(i-1)*16 + 7] + d[(i-1)*16 + 8] == 1)               


    @addNLConstraint(mdl, d[(i-1)*16 + 9]*(-(x[i] + Lf*cos(psi[i]) - width/2*sin(psi[i])) + xr)
                   + d[(i-1)*16 + 10]*(y[i] + Lf*sin(psi[i]) + width/2*cos(psi[i]) - yt) >= 0)
    @addNLConstraint(mdl, d[(i-1)*16 + 9] + d[(i-1)*16 + 10] == 1)               

    @addNLConstraint(mdl, d[(i-1)*16 + 11]*(-(x[i] + Lf*cos(psi[i]) + width/2*sin(psi[i]))+ xr)
                   + d[(i-1)*16 + 12]*(y[i] + Lf*sin(psi[i]) - width/2*cos(psi[i]) - yt) >= 0)
    @addNLConstraint(mdl, d[(i-1)*16 + 11] + d[(i-1)*16 + 12] == 1)               

    @addNLConstraint(mdl, d[(i-1)*16 + 13]*(-(x[i] - Lr*cos(psi[i]) + width/2*sin(psi[i]))+xr) 
                   + d[(i-1)*16 + 14]*(y[i] - Lr*sin(psi[i]) - width/2*cos(psi[i]) - yt) >= 0)
    @addNLConstraint(mdl, d[(i-1)*16 + 13] + d[(i-1)*16 + 14] == 1)               

    @addNLConstraint(mdl, d[(i-1)*16 + 15]*(-(x[i] - Lr*cos(psi[i]) - width/2*sin(psi[i]))+xr) 
                   + d[(i-1)*16 + 16]*(y[i] - Lr*sin(psi[i]) + width/2*cos(psi[i]) - yt) >= 0)
    @addNLConstraint(mdl, d[(i-1)*16 + 15] + d[(i-1)*16 + 16] == 1)               

    @addNLConstraint(mdl, x_min <= x[i] + Lf*cos(psi[i]) - width/2*sin(psi[i]) <= x_max)
    @addNLConstraint(mdl, x_min <= x[i] + Lf*cos(psi[i]) + width/2*sin(psi[i]) <= x_max)
    @addNLConstraint(mdl, x_min <= x[i] - Lr*cos(psi[i]) + width/2*sin(psi[i]) <= x_max)
    @addNLConstraint(mdl, x_min <= x[i] - Lr*cos(psi[i]) - width/2*sin(psi[i]) <= x_max)

    @addNLConstraint(mdl, y_min <= y[i] + Lf*sin(psi[i]) + width/2*cos(psi[i]) <= y_max)
    @addNLConstraint(mdl, y_min <= y[i] + Lf*sin(psi[i]) - width/2*cos(psi[i]) <= y_max)
    @addNLConstraint(mdl, y_min <= y[i] - Lr*sin(psi[i]) - width/2*cos(psi[i]) <= y_max)
    @addNLConstraint(mdl, y_min <= y[i] - Lr*sin(psi[i]) + width/2*cos(psi[i]) <= y_max)
    
end

# status update
println("initial solve ...")
solve(mdl)
println("finished initial solve!")

function SE_callback(msg::Z_KinBkMdl)
    global psi_offset
    global read_yaw0
    global x_ref
    global y_ref
    # update mpc initial condition 
    setValue(x0,    msg.x)
    setValue(y0,    msg.y)
    setValue(psi0,  msg.psi + psi_offset)
    setValue(v0,    msg.v)
    if read_yaw0 == 0
        read_yaw0 = 1
        psi_offset = -psi0
        # set_targets(psi_ref)
        x_ref   = x_int
        y_ref   = y_int
    end

    x_curr = getValue(x0)
    y_curr = getValue(y0)
    psi_curr = getValue(psi0)
    v_curr = getValue(v0)
    if (x_curr - x_ref)^2 + (y_curr - y_ref)^2 + (psi_curr - psi_ref)^2 + (v_curr - v_ref)^2 <= 0.05
        x_ref   = x_final
        y_ref   = y_final
    end
end

function set_targets(psi_ref)
   offset_angle_int = atan(y_int/x_int)    
   offset_angle_final = atan(y_final/x_final)
   x_int0 = x_int
   x_final0 = x_final
   x_int = x_int0*cos(psi_ref + offset_angle_int)
   y_int = x_int0*sin(psi_ref + offset_angle_int)
   x_final = x_final0*cos(psi_ref + offset_angle_final)
   y_final = y_final0*sin(psi_ref + offset_angle_final)
end
 
function angle_2_servo(x)
     x = x-2
     u = 92.0558 + 1.8194*x - 0.0104*x^2
     return u
 end

function accel_2_pwm(a)
    pwm = nearest_pwm(a)
    if a > 0
        pwm = max(100, pwm)
    else
        pwm = min(87, pwm)
    end
    return pwm
end

function nearest_pwm(a_des)
    best_idx = 0
    min_err = 10
    for i = [1:length(accel_range)]
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
    pub_steer = Publisher("servo_pwm", ECU, queue_size=10)
    pub_motor = Publisher("motor_pwm", ECU, queue_size=10)
    pub_sgn   = Publisher("vel_sgn", vel_sgn, queue_size=10)
    pub_ecu   = Publisher("ecu", ECU, queue_size=10)
    s1  = Subscriber("state_estimate", Z_KinBkMdl, SE_callback, queue_size=10)

    # BARC has two modes: forward and reverse, with hysteresis.
    # Default is forward mode. pwm > 90 passes given pwm to motor. Applying 66 < pwm < 90 only brakes. 
    # Must send pwm < 67 (full brakes), then 88 < pwm < 94 (neutral), to get to reverse mode
    
    # TODO: change forward_mode to Bool. Requires some overhead with catkin_make
    sign = vel_sgn()
    sign.forward_mode = 1
    loop_rate = Rate(10)
    
    is_parked = 0
    while !is_shutdown() && is_parked == 0
        # run mpc, publish command
        status = solve(mdl)
        if status != :Optimal
            break
        end

        # get optimal solutions
        a_opt   = getValue(a[1])
        d_f_opt = getValue(d_f[1])

        esc_cmd = accel_2_pwm(a_opt)
        servo_cmd = angle_2_servo(d_f_opt*180/pi)
   
        vel = getValue(v0)
        if (abs(vel) < 0.3) && (esc_cmd < 88 || esc_cmd > 94)
            if esc_cmd < 88 && sign.forward_mode == 1
                sign.forward_mode = 0
                publish(pub_sgn, sign)
                cmd = ECU(66, servo_cmd)
                publish(pub_ecu, cmd)
                rossleep(loop_rate)

                cmd = ECU(90, servo_cmd)
                publish(pub_ecu, cmd)
                rossleep(loop_rate)
            elseif esc_cmd > 94 && sign.forward_mode == 0
                sign.forward_mode = 1
                publish(pub_sgn, sign)
            end

            cmd = ECU(esc_cmd, servo_cmd)
            publish(pub_ecu, cmd)
        else
            cmd = ECU(esc_cmd, servo_cmd)

            # publish commands
            # arduino_interface handles rest of ECU publication
            publish(pub_ecu, cmd)

            rossleep(loop_rate)
        end
        x_curr = getValue(x0)
        y_curr = getValue(y0)
        psi_curr = getValue(psi0)
        v_curr = getValue(v0)
        if (x_curr - x_final)^2 + (y_curr - y_final)^2 + (psi_curr - psi_final)^2 + (v_curr - v_final)^2 <= 0.05
            is_parked = 1
        end
    end
    cmd = ECU(90,90)
    publish(pub_ecu, cmd)
end

if !isinteractive()
    main()
end
