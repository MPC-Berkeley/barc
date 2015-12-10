#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using geometry_msgs.msg

using JuMP
using Ipopt
include("/home/odroid/catkin_ws/src/barc/src/VhMdl.jl")

# define parameters
a           = 0.15      # distance from CoG to front axel
b           = 0.10      # distance from CoG to back axel
m           = 0.25      # mass [kg]
I_z         = 0.15      # moment of inertia about z-axis [mg*m^2]
dt          = 0.10      # sampling time

# reference states and reference inputs
v_x         = 4.5
v_y_ref     = 0.665
r_ref       = -2.122
d_F_ref     = 0.0394

vh          = VhMdl(a, b, m, I_z, v_x, dt)
TM_F        = [25, 1.5, -0.981]
TM_R        = [25, 1.5, -1.4715]
tm          = TMmdl(TM_F, TM_R);
(B1,C1,D1)  = TM_F
(B2,C2,D2)  = TM_R

N           = 5
mdl         = Model(solver = IpoptSolver(print_level = 3))

# define decision variables
@defVar(mdl, v_y[1:(N+1)])
@defVar(mdl, r[1:(N+1)])
@defVar(mdl, d_F[1:N])

# Define objective function
@setNLObjective(mdl, Min, (r[N+1]/r_ref - 1)^2 + (v_y[N+1]/v_y_ref - 1)^2  + 0.5*(d_F[N]/d_F_ref - 1)^2 )

# Define constraints
# 1. Compute lateral forces from slip angles ang pajecka tire model
@defNLExpr(a_F[i = 1:N],   atan((v_y[i] + a*r[i])/v_x) - d_F[i])
@defNLExpr(a_R[i = 1:N],   atan((v_y[i] - b*r[i])/v_x) )
@defNLExpr(FyF[i = 1:N],   D1*sin(C1*atan(B1 * a_F[i])) )
@defNLExpr(FyR[i = 1:N],   D2*sin(C2*atan(B2 * a_R[i])) )

# 2. vehicle dynamic constraints
v_y0    = [Float64(0)]
r0      = [Float64(0)]
@addNLConstraint(mdl, v_y[1] == v_y0[1])
@addNLConstraint(mdl, r[1] == r0[1])
for i in 1:N
  @addNLConstraint(mdl, v_y[i+1] == v_y[i] + dt*(-r[i]*v_x +  (FyF[i]*cos(d_F[i])  +  FyR[i])/m) )
  @addNLConstraint(mdl,   r[i+1] == r[i]   +            dt*( a*FyF[i]*cos(d_F[i]) - b*FyR[i])/I_z )
  @addConstraint(mdl, -0.5 <=  d_F[i] <= 0.5 )
end
println("initial solve ...")
solve(mdl)
println("finished initial solve!")

function callback(msg::Vector3, pub_obj::Publisher{Vector3})
    v_y0[1] = msg.y
    r0[1]   = msg.z
end

function loop(pub_obj)
    loop_rate = Rate(10)
    while ! is_shutdown()
        solve(mdl)
        d_F_opt   = getValue(d_F[1])
        cmd_sig = Vector3(d_F_opt, 0.0, 0.0)
        publish(pub_obj, cmd_sig)
        rossleep(loop_rate)
    end
end

function main()
    init_node("NMPC_control")
    pub = Publisher{Vector3}("NMPC_ctrl_sig", queue_size=10)
    sub = Subscriber{Vector3}("state_estimate", callback, (pub,), queue_size=10)
    loop(pub)
end

if ! isinteractive()
    main()
end
