#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using geometry_msgs.msg

#=
using JuMP
using Ipopt
include("VhMdl.jl")

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

=#

X0    = [Float64(0)]
Y0    = [Float64(0)]
phi0  = [Float64(0)]
v_x0  = [Float64(0)]
v_y0  = [Float64(0)]
r0    = [Float64(0)]

phiSensor0  = [Float64(0)]
rSensor0 = [Float64(0)]
err = [Float64(0)];
angleSet = [Int32(0)]
angleRef = [Float64(0)]
freq = 10;
integralOld = [Float64(0)];

function callback1(msg::Vector3, pub_obj::Publisher{Vector3})
 
    v_x0[1] = msg.x
    v_y0[1] = msg.y
    r0[1]   = msg.z


end

function callback2(msg::Vector3, pub_obj::Publisher{Vector3})
 
    phiSensor0[1] = msg.x
    rSensor0[1] = msg.y

    if angleSet[1] == 0
        angleRef[1] = phiSensor0[1];
        angleSet[1] = 1;
    end
    
end

function callback3(msg::Vector3, pub_obj::Publisher{Vector3})
 
    X0[1] = msg.x
    Y0[1] = msg.y
    phi0[1]   = msg.z
end

# [deg] -> [PWM]
function angle_2_servo(x)
    u = 92.0558 + 1.8194*x - 0.0104*x^2;
    return u;
end

function PIcontrol(err)

    Pgain = 50;
    Igain = 5;

    proportional = Pgain*err;
    integral = integralOld[1] + 1/freq* Igain *err;
    integralOld[1] = integral;

    return (proportional + integral)

end


function rateLimiter(x, xOld, limit, dt)

    if (x - xOld) > dt*limit
        y = xOld + dt*limit;
    elseif (x - xOld) < -dt*limit
           y = xOld - dt*limit;
        else
           y = x;    
    end
    return y;

end

function main()
    init_node("MPC_controller")
    pub = Publisher{Vector3}("ecu_cmd", queue_size=10)
    # (topic, callback, callback_arguments, queue_size) 
    sub1 = Subscriber{Vector3}("state_estimate", callback1, (pub,), queue_size=10)
    sub2 = Subscriber{Vector3}("angle_info", callback2, (pub,), queue_size=10)
    sub3 = Subscriber{Vector3}("position_info", callback3, (pub,), queue_size=10)
    loop_rate = Rate(freq);
    timeGlobal = 0.0;
    timeStart = 2.0;
    timeTurn = 5.0;
    steeringAngle = 0;
    motorCmd = 90;
    err = 0;

       while ! is_shutdown()
        # solve(mdl)
        
        #=
        println("-------")
        print("Vx = ")
        println( v_x0[1] )
        print("Vy = ")
        println( v_y0[1] )
        print("r = ")
        println( r0[1] )
        print("X = ")
        println( X0[1] )
        print("Y = ")
        println( Y0[1] )
        print("phi = ")
        println( phiSensor0[1] )
        print("time = ")
        println( timeGlobal )
        println("-------")
        =#

        
        if timeGlobal > 2        
            motorCmd = 96;
        end

        
        if timeGlobal > 5
            steeringAngle = 10;
        end
        
        
        if timeGlobal > 10
            steeringAngle = -10;
        end
        


        if timeGlobal > 15
            motorCmd = 90;
        end
       
        
        if angleSet[1] == 1
            err = angleRef[1] - phiSensor0[1];
        end

        #steeringAngle = PIcontrol(err);
        
        servoCmd = angle_2_servo(steeringAngle);
        cmd_sig = Vector3(motorCmd, servoCmd, steeringAngle*pi/180);
        publish(pub, cmd_sig)
        
        timeGlobal = timeGlobal + 1/freq; 



        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
