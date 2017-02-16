#!/usr/bin/env julia
using RobotOS

@rosimport barc.msg: ECU, Encoder, Ultrasound, Z_KinBkMdl
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
@rosimport sensor_msgs.msg: LaserScan
@rosimport marvelmind_nav.msg: hedge_pos
rostypegen()

using barc.msg
using data_service.msg
using geometry_msgs.msg
using JuMP, Ipopt
using sensor_msgs.msg
using marvelmind_nav.msg
# using PyPlot
using PyCall
# using Plots

type Obstacle
    posX::Float64
    posY::Float64
    velX::Float64
    velY::Float64
    posList::Dict
    velList::Dict
    index::Int
end

print("enteriing julia file..... ")

# define the model and environment parameters
width=0.19/2 # haL_b of car's width
L_a=0.21
L_b=0.19
L = L_a + L_b
car_size = [L,L_a,L_b,width]
r2 = max(((L_a+0.12)^2+(width+0.08)^2),((L_b+0.12)^2+(width+0.08)^2))
Lane_width = 0.38
upper_shoulder = 1.5*Lane_width
lower_shoulder = -0.5*Lane_width
num_Obs = 2

# simulation time settings
dt = 0.1
simTime=30 # total simulation time
Tsim=Int(simTime/dt)
horizonLength = 30 #horizon length of long-horizon MPC
horizonLength_MPC = 5 # horizon length of low-level controller (MPC-based)

# states dimension and number of outputs
n = 4
m = 2

# physical constraints of the car model
vmax = 1.0
vmin = 0.0
zmax=[Inf;1.5*Lane_width;pi/3;vmax]
zmin=[-Inf;-0.5*Lane_width;-pi/3;vmin]
umax=[0.5 pi/6]
umin=[-1 -pi/6]

# variables definition, those parameters should be updated in the callback functions triggered by the encoder, gps and LiDAR
u_pre = [0.0 0.0]
z_t = [0.0; 0.0; 0.00000001; 0.0] # current state of the car
Obstacles = Dict()
Obstacle_X = rand(-4:0.01:10)
vel_initial = rand(-1.0:0.01:1.0)
posList_initial = Dict(1=>reshape([Obstacle_X, Lane_width],(1,2)))
velList_initial = Dict(1=>reshape([vel_initial, 0],(1,2)))
Obstacles[1] = Obstacle(Obstacle_X, Lane_width, vel_initial, 0.0, posList_initial, velList_initial, 1)
Obstacle_X = rand(-4:0.01:10)
vel_initial = rand(-1.0:0.01:1.0)
posList_initial = Dict(1=>reshape([Obstacle_X, 0],(1,2)))
velList_initial = Dict(1=>reshape([vel_initial, 0],(1,2)))
Obstacles[2] = Obstacle(Obstacle_X, Lane_width, vel_initial, 0.0, posList_initial, velList_initial, 1)

# note that the maximum scanning frequency of LiDAR is 10Hz, which means that we can detect the obstacle in every single step.
ref_NN = zeros(1,10)
obj_log = Dict()

using MAT
x_unit = [0 0.2 0.4 0.6 0.8 1.0 1.35 1.7 2.05 2.4]
Net = matread("/home/mpcubuntu/barc/workspace/src/barc/src/Weights_net_twoObs_phy_first5_out_xy_vlim_Jan20compVmaxNegSpeed.mat")

function forwardNN()
    num_features = num_Obs*4+2+size(x_unit,2)+1+1
    feature = zeros(1, num_features);
    obsPos  = zeros(1, num_Obs*4+2);
    carPosX = z_t[1]
    #print("Current car position:")
    #print(z_t[1:2])
    #print("\n")
    carPosY = z_t[2]
    carYaw  = z_t[3]
    carV    = z_t[4]

    carBeta = atan(L_b*tan(u_pre[1,2])/(L_a+L_b))
    targetX = carPosX + vmax * 30 * dt;
    for obs_i = 1:num_Obs
        obsPos[(obs_i-1)*4+1] = carPosX - Obstacles[obs_i].posX;
        obsPos[(obs_i-1)*4+2] = carPosY - Obstacles[obs_i].posY;
        obsPos[(obs_i-1)*4+3] = carV*cos(carYaw + carBeta) - Obstacles[obs_i].velX;
        obsPos[(obs_i-1)*4+4] = carV*sin(carYaw + carBeta) - Obstacles[obs_i].velY;
    end
    obsPos[4*num_Obs+1] = 1.5*Lane_width - carPosY;
    obsPos[4*num_Obs+2] = -(-0.5*Lane_width - carPosY);

    feature[1:4*num_Obs+2] = obsPos;
    feature[num_features - size(x_unit, 2)-1:num_features-1-1] = carPosY + x_unit * tan(carYaw);
    feature[num_features-1] = carPosX + carV * cos(carYaw + carBeta) * 30 * dt - targetX;
    feature[num_features] = carV - vmax;
    #print(feature)

    feature_normalized = reshape(feature, 1, 22)
    feature_normalized[1] = feature_normalized[1]/3.0
    feature_normalized[5] = feature_normalized[5]/3.0
    feature_normalized = feature_normalized'

    input_range = Net["input_range"]
    Bias = Net["Bias"]
    IW = Net["IW"]
    LW = Net["LW"]
    output_range = Net["output_range"]
    x_in = (2*feature_normalized-input_range[:,1]-input_range[:,2])./(input_range[:,2]-input_range[:,1])
    temp = LW*tanh(IW*x_in+Bias[1])+Bias[2]
    ref_NN = reshape((temp.*(output_range[:,2]-output_range[:,1])+output_range[:,2]+output_range[:,1])/2, 1, 10)
    ref_NN_x = reshape(ref_NN[1:5], 1, 5)*3 + repmat([carPosX], 1, 5)
    ref_NN_y = reshape(ref_NN[6:10], 1, 5)
    ref_NN = [ref_NN_x ref_NN_y]
    return ref_NN
end

function getPos(ball::Obstacle, t)
    prePosX = ball.posX + ball.velX * t
    prePosY = ball.posY + ball.velY * t
    return (prePosX, prePosY)
end

function solve_short_horizon_MPC()
    # maximum distance for obstacle avoidance, use an elipsoid to circle the car
    r_long = max((L_a+0.12),(L_b+0.12))
    r_short = width+0.08
    r2 = r_long^2 + r_short^2
    T = horizonLength_MPC
    posT = reshape(deepcopy(ref_NN), 2, horizonLength_MPC)
    method = 1 # 1: soft constraints; 2: hard constraints

    # define the optimization problem
    # model and variables
    mdl = Model(solver=IpoptSolver(print_level=0))
    @variable(mdl, zmin[i] <= z[i=1:n,t=0:T] <= zmax[i])
    @variable(mdl, umin[i] <= u[i=1:m,t=0:T-1] <= umax[i])

    # Cost function
    obj= 0
    obj = obj + 5000*(u[1,0]-u_pre[1])^2 + 5000*(u[2,0]-u_pre[2])^2
    for t=0:T-2
        if t>0
            obj = obj + 100*(z[2,t]-posT[2,t])^2 + 100*(z[1,t]-posT[1,t])^2# + (z[4,t]-zmax[4])^2
        end
        obj = obj + 5000*(u[1,t+1]-u[1,t])^2 + 5000*(u[2,t+1]-u[2,t])^2
#         for i = 1:m
#         end
    end
    obj = obj + 100*(z[2,T-1]-posT[2,T-1])^2 + 100*(z[1,T-1]-posT[1,T-1])^2 + 100*(z[2,T]-posT[2,T])^2 + 100*(z[1,T]-posT[1,T])^2# + (z[4,T-1]-zmax[4])^2 + (z[4,T]-zmax[4])^2


    # handling constraints
    # Method 1: define slack variable, soft constraint
    if method==1
        @variable(mdl, s[i=1:num_Obs+1,t=0:T-1]>=0)
        for i=1:num_Obs
            for t=0:T-1
                obj = obj+10000000*s[i,t]
            end
        end
        for t=0:T-1
            obj = obj+1000000*s[num_Obs+1,t]^2
        end
        @objective(mdl, Min, obj)
    else
        # method 2: hard constraints
        @objective(mdl, Min, obj)
    end

    # constrains from car kinematics
    for t = 0:T-1
        @NLconstraint(mdl, z[1,t+1] == z[1,t] + dt*z[4,t]*cos(z[3,t]+atan(L_b*tan(u[2,t])/(L_a+L_b))))
        @NLconstraint(mdl, z[2,t+1] == z[2,t] + dt*z[4,t]*sin(z[3,t]+atan(L_b*tan(u[2,t])/(L_a+L_b))))
        @NLconstraint(mdl, z[3,t+1] == z[3,t] + dt*z[4,t]/(L_b+L_a)*tan(u[2,t])*cos(atan(L_b*tan(u[2,t])/(L_a+L_b))))
        @NLconstraint(mdl, z[4,t+1] == z[4,t] + dt*u[1,t])
    end

    # static obstacle avoidance constraints
    # basically, this is to constraint the car's moving space
    if (0<=1)
        # Method 1: add cosntrain for moving ball avoidance
        for t=0:T-3
            @NLconstraint(mdl, ((z[1,t+2]-2*z[1,t+1]+z[1,t])/dt^2)^2 + ((z[2,t+2]-2*z[2,t+1]+z[2,t])/dt^2)^2 <= umin[1]^2)
        end

        for t = 0:T-1
            @NLconstraint(mdl, z[2,t]+L_a*sin(z[3,t])-width*cos(z[3,t]) >=zmin[2])
            @NLconstraint(mdl, z[2,t]-L_b*sin(z[3,t])-width*cos(z[3,t]) >=zmin[2])
            @NLconstraint(mdl, z[2,t]+L_a*sin(z[3,t])+width*cos(z[3,t]) <=zmax[2])
            @NLconstraint(mdl, z[2,t]-L_b*sin(z[3,t])+width*cos(z[3,t]) <=zmax[2])
            if method==1
                @constraint(mdl, s[num_Obs+1,t] >=0)
                @constraint(mdl, s[num_Obs+1,t]>=-z[2,t])
                for i=1:num_Obs
                    xObs, yObs = getPos(Obstacles[i], t*dt)
                    @constraint(mdl, s[i,t] >=0)
                    @NLconstraint(mdl, s[i,t]>=r2-(z[1,t]-xObs)^2-(z[2,t]-yObs)^2)
                end
            else
                for i=1:num_Obs
                    xObs, yObs = getPos(Obstacles[i], t*dt)
                    xObs, yObs = (3.0,0)
                    @NLconstraint(mpc, r2-(z[1,t]-xObs)^2-(z[2,t]-yObs)^2<=0)
                end
            end
        end
    end

    # Initial conditions
    @constraint(mdl, z[:,0] .== z_t)

    # Solve the NLP
    status = solve(mdl)

    ref_MPC = getvalue(z)
    #print("The optimal safe reference is: [x=%.4f, y=%.4f]", ref_MPC[1,1], ref_MPC[2,1] )
    #print("\n")
    # Return the first control plan(receding horizon strategy)
    print(getValue(u[:,0]))
    #return getValue(u[:,0]), getValue(z[:,1])
#     print(getvalue(z))
    return getvalue(u), ref_MPC, status, getvalue(obj)
end

function SE_callback(msg::Z_KinBkMdl)
    # update mpc initial condition
    z_t[1] = msg.x
    z_t[2] = msg.y
    z_t[3] = msg.psi
    z_t[4] = msg.v
end

function LIDAR_callback(msg::LaserScan)
    # Obstacles update
end

function gps_callback(msg::hedge_pos)
    #print(msg.x_m)
    #print(msg.y_m)
end

samples_per_scan = 10
function main()
    # initiate node, set up publisher / subscriber topics
    init_node("NN_MPC")
    print("launched this node ..... ")
    pub = Publisher("ecu", ECU, queue_size=10)
    sub = Subscriber("state_estimate", Z_KinBkMdl, SE_callback, queue_size=10)
    sub = Subscriber("scan", LaserScan, LIDAR_callback, queue_size=samples_per_scan)
    sub = Subscriber("hedge_pos", hedge_pos, gps_callback, queue_size=10)
    loop_rate = Rate(10)

    run_index = 1

    while ! is_shutdown()
        # run forwardNN, get the reference given by network prediction
        ref_NN = forwardNN()
        #print("The new reference given by network is: [x=%.4f, y=%.4f]", ref_NN[1], ref_NN[6])
        #print("\n")

        # run NNMPC, publish command
        u_vec, z_vec, status, obj_log[run_index] = solve_short_horizon_MPC()

        # get optimal solutions
        a_opt   = u_vec[1,0]
        d_f_opt = u_vec[2,0]
        cmd = ECU(a_opt, d_f_opt)
        u_pre[1] = a_opt
        u_pre[2] = d_f_opt

        # publish commands
        publish(pub, cmd)
        run_index += 1
        rossleep(loop_rate)
    end
end


if ! isinteractive()
    main()
end
