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
@rosimport barc.msg: ECU, pos_info, Encoder, Ultrasound, Z_KinBkMdl, Logging
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using JuMP
using Ipopt


# define model parameters
L_a     = 0.125         # distance from CoG to front axel
L_b     = 0.125         # distance from CoG to rear axel
dt      = 0.1           # time step of system
coeffCurvature   = [0,0,0,0]

# preview horizon
N       = 10

# define targets [generic values]
v_ref   = 0.8

# define objective function values
c_ey = 50
c_ev = 10
c_epsi = 5
c_df = 0.3
c_a = 0.2
c_ey_f = 0
c_ev_f = 0
c_epsi_f = 0

# define decision defVars 
# states: position (x,y), yaw angle, and velocity
# inputs: acceleration, steering angle 
println("Creating kinematic bicycle model ....")
mdl     = Model(solver = IpoptSolver(print_level=3,max_cpu_time=0.1)) # ,linear_solver="ma97",print_user_options="yes"))

@defVar( mdl, s[1:(N+1)] )
@defVar( mdl, ey[1:(N+1)] )
@defVar( mdl, epsi[1:(N+1)] )
@defVar( mdl, 0.0 <= v[1:(N+1)] <= 3.0 )
@defVar( mdl, 2.0 >= a[1:N] >= -1.0 )
@defVar( mdl, -0.6 <= d_f[1:N] <= 0.6 )

# define objective function
@setNLObjective(mdl, Min, sum{c_ey*ey[i]^2+c_ev*(v[i]-v_ref)^2+c_epsi*epsi[i]^2+c_df*d_f[i]^2+c_a*a[i]^2,i=1:N} + c_ey_f*ey[N+1]^2 + c_ev_f*(v[N+1]-v_ref)^2 + c_epsi_f*epsi[N+1]^2)

# define constraints
# define system dynamics
# Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series,
#               Spring, 2011, page 26

@defNLParam(mdl, s0      == 0); @addNLConstraint(mdl, s[1]      == s0);
@defNLParam(mdl, ey0     == 0); @addNLConstraint(mdl, ey[1]     == ey0);
@defNLParam(mdl, epsi0   == 0); @addNLConstraint(mdl, epsi[1]   == epsi0 );
@defNLParam(mdl, v0      == 0); @addNLConstraint(mdl, v[1]      == v0);
@defNLParam(mdl, coeff[i=1:length(coeffCurvature)]==coeffCurvature[i]);
@defNLExpr(mdl, c[i = 1:N],    coeff[1]*s[i]^3+coeff[2]*s[i]^2+coeff[3]*s[i]+coeff[4])
@defNLExpr(mdl, bta[i = 1:N],    atan( L_a / (L_a + L_b) * tan( d_f[i]) ) )

@defNLExpr(mdl, dsdt[i = 1:N], v[i]*cos(epsi[i]+bta[i])/(1-ey[i]*c[i]))
for i in 1:N
    @addNLConstraint(mdl, s[i+1]     == s[i]       + dt*dsdt[i]  )
    @addNLConstraint(mdl, ey[i+1]    == ey[i]      + dt*v[i]*sin(epsi[i]+bta[i])  )
    @addNLConstraint(mdl, epsi[i+1]  == epsi[i]    + dt*(v[i]/L_a*sin(bta[i])-dsdt[i]*c[i])  )
    @addNLConstraint(mdl, v[i+1]     == v[i]       + dt*(a[i]  - 0.63 *abs(v[i])*v[i])  )
end


# status update
println("initial solve ...")
solve(mdl)
println("finished initial solve!")


# write info in logfile
currenttime = now()
f = open("/home/odroid/rosbag/MPClog_$currenttime.txt","w")
byteswritten=write(f,"MPC LOG, created $currenttime\n=========================\nv_ref = $v_ref\nN = $N\nc_ey = $c_ey\nc_ev = $c_ev\nc_epsi = $c_epsi\nc_df = $c_df\nc_a = $c_a\nc_ey_f = $c_ey_f\nc_ev_f = $c_ev_f\nc_epsi_f = $c_epsi_f")
close(f)

if byteswritten>0
    println("Successfully written to Logfile MPClog_$currenttime.txt")
else
    println("Problem with Logfile")
end


function SE_callback(msg::pos_info)
    # update mpc initial condition 
    setValue(s0,     msg.s)
    setValue(ey0,    msg.ey)
    setValue(epsi0,  msg.epsi)
    setValue(v0,     msg.v)
    setValue(coeff,  msg.coeffCurvature)
end

function main()
    # initiate node, set up publisher / subscriber topics
    init_node("mpc_traj")
    pub = Publisher("ecu", ECU, queue_size=10)
    pub2 = Publisher("logging", Logging, queue_size=10)
    s1  = Subscriber("pos_info", pos_info, SE_callback, queue_size=10)
    loop_rate = Rate(10)
    cmdcount = 0
    failcount = 0
    while ! is_shutdown()
        # run mpc, publish command
        tic()
        status = solve(mdl)
        solvetime = toc()
        if status == Symbol("Optimal")
            # get optimal solutions
            a_opt   = getValue(a[1])
            d_f_opt = getValue(d_f[1])
            cmd = ECU(a_opt, d_f_opt)     
            # publish commands
            if cmdcount>10      # ignore first 10 commands since MPC often stagnates during the first seconds (why?)
                publish(pub, cmd)
            end
            failcount = 0
        else
            failcount = failcount + 1
            if failcount >= 5       # if at least 5 unsolved problems in a row
                cmd = ECU(0,0)      # stop car
                publish(pub,cmd)
            end
        end
        println("Solve Status: ", string(status), "\nSolve Time: ", solvetime,"\n\n")
        loginfo = Logging(getObjectiveValue(mdl),string(status),solvetime,getValue(s),getValue(ey),getValue(epsi),getValue(v),getValue(a),getValue(d_f))
        publish(pub2, loginfo)
        cmdcount = cmdcount + 1
        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
