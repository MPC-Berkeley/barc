# VARIOUS TYPES FOR CALCULATIONS

type LapStatus
    currentLap::Int64       # current lap number
    currentIt::Int64        # current iteration in current lap
    switchLap::Bool
    nextLap::Bool
    s_lapTrigger::Float64
end

# Structure of coeffConst:
# 1st dimension is the polynomial coefficient
# 2nd dimension specifies the state (1 = eY, 2 = ePsi, 3 = v or 1 = xDot, 2 = yDot, 3 = psiDot, 4 = ePsi, 5 = eY)
# 3th dimension specifies one of the two lap numbers between which are iterated

type MpcCoeff           # coefficients for trajectory approximation
    c_Vx::Array{Float64,2}
    c_Vy::Array{Float64,2}
    c_Psi::Array{Float64,2}
    MpcCoeff(c_Vx=zeros(10,3),c_Vy=zeros(10,4),c_Psi=zeros(10,3)) = new(c_Vx, c_Vy, c_Psi)
end

type OldTrajectory      # information about previous trajectories
    oldTraj::Array{Float64}             # contains all states over all laps
    oldInput::Array{Float64}            # contains all inputs
    oldTimes::Array{Float64}            # contains times related to states and inputs
    oldCost::Array{Int64}               # contains costs of laps
    count::Array{Int64}                 # contains the counter for each lap
    idx_start::Array{Int64}             # index of the first measurement with s > 0
    idx_end::Array{Int64}               # index of the last measurement with s < s_target
    OldTrajectory(oldTraj=Float64[],oldInput=Float64[],oldTimes=Float64[],oldCost=Float64[],count=Int64[],idx_start=Int64[],idx_end=Int64[]) = new(oldTraj,oldInput,oldTimes,oldCost,count,idx_start,idx_end)
end

type SolHistory
    u::Array{Float64,4}
    z::Array{Float64,4}
    cost::Array{Float64,1}
    function SolHistory(bufferSize::Int64,N::Int64,n_state::Int64,n_lap::Int64)
        solHistory=new()
        solHistory.u=zeros(bufferSize,n_lap,N,2)
        solHistory.z=zeros(bufferSize,n_lap,N+1,n_state)
        solHistory.cost=zeros(n_lap)
        return solHistory
    end
end

# To make the data structure to be compatible with my own simulation code, idx_start and idx_end in SafeSetData are acturally not used, since I am using other way to cantacate the laps.
type SafeSetData
    oldSS::Array{Float64}           # contains data from previous laps usefull to build the safe set
    cost2target::Array{Float64}     # cost to arrive at the target, i.e. how many iterations from the start to the end of the lap
    oldCost::Array{Int64}               # contains costs of laps
    count::Array{Int64}                 # contains the counter for each lap
    idx_start::Array{Int64}             # index of the first measurement with s > 0
    idx_end::Array{Int64}               # index of the last measurement with s < s_target
    oldSS_xy::Array{Float64}
    SafeSetData(oldSS=Float64[],cost2target=Float64[],oldCost=Int64[],count=Int64[],idx_start=Int64[],idx_end=Int64[],oldSS_xy=Float64[]) =
    new(oldSS,cost2target,oldCost,count,idx_start,idx_end,oldSS_xy)
end

type SelectedStates                 # Values needed for the convex hull formulation
    selStates::Array{Float64}       # selected states from previous laps ...
    statesCost::Array{Float64}      # ... and their related costs
    Np::Int64                       # number of states to select from each previous lap
    Nl::Int64                       # number of previous laps to include in the convex hull
    SelectedStates(selStates=Float64[],statesCost=Float64[],Np=10,Nl=2) = new(selStates,statesCost,Np,Nl)
end

type MpcParams          # parameters for MPC solver
    N::Int64
    nz::Int64
    Q::Array{Float64,1}
    Q_term::Array{Float64,1}
    R::Array{Float64,1}
    vPathFollowing::Float64
    QderivZ::Array{Float64,1}
    QderivU::Array{Float64,1}
    Q_term_cost::Float64
    delay_df::Int64
    delay_a::Int64
    Q_lane::Float64                # weight on the soft constraint for the lane
    Q_vel::Float64                 # weight on the soft constraint for the maximum velocity
    Q_slack::Array{Float64,1}               # weight on the slack variables for the terminal constraint
    Q_obs::Array{Float64}          # weight used to esclude some of the old trajectories from the optimization problem
    MpcParams(N=0,nz=0,Q=Float64[],Q_term=Float64[],R=Float64[],vPathFollowing=1.0,QderivZ=Float64[],QderivU=Float64[],Q_term_cost=1.0,delay_df=0,delay_a=0,Q_lane=1.0,Q_vel=1.0,Q_slack=Float64[],Q_obs=Float64[])=
    new(N,nz,Q,Q_term,R,vPathFollowing,QderivZ,QderivU,Q_term_cost,delay_df,delay_a,Q_lane,Q_vel,Q_slack,Q_obs)
end

type PosInfo            # current position information
    s::Float64
    s_target::Float64
    PosInfo(s=0,s_target=0) = new(s,s_target)
end

type MpcSol             # MPC solution output
    a_x::Float64
    d_f::Float64
    solverStatus::Symbol
    u::Array{Float64}
    z::Array{Float64}
    cost::Array{Float64}
    eps_alpha::Array{Float64}
    costSlack::Array{Float64}
    MpcSol(a_x=0.0,d_f=0.0,solverStatus=Symbol(),u=Float64[],z=Float64[],cost=Float64[],eps_alpha=Float64[],costSlack=Float64[]) = new(a_x,d_f,solverStatus,u,z,cost,eps_alpha,costSlack)
end

type ModelParams
    l_A::Float64
    l_B::Float64
    m::Float64
    I_z::Float64
    dt::Float64
    u_lb::Array{Float64}  
    u_ub::Array{Float64}
    z_lb::Array{Float64}
    z_ub::Array{Float64}
    c0::Array{Float64}
    c_f::Float64
    ModelParams(l_A=0.25,l_B=0.25,m=1.98,I_z=0.24,dt=0.1,u_lb=Float64[],u_ub=Float64[],z_lb=Float64[],z_ub=Float64[],c0=Float64[],c_f=0.0) = new(l_A,l_B,m,I_z,dt,u_lb,u_ub,z_lb,z_ub,c0,c_f)
end

# THIS IS THE CORRESPONDING TRACK DATA IN JULIA
# WE NEED TO DEFINE THEM BOTH IN PYTHON AND JULIA

function add_curve(theta::Array{Float64,1},curvature::Array{Float64,1},num_point,angle::Float64,ds::Float64)
    d_theta = 0
    curve = 2*sum(1:num_point/2)#+num_point/2
    for i=1:num_point
        if i <= num_point/2
            d_theta = d_theta + angle / curve
        elseif i == num_point/2+1
            d_theta = d_theta
        else
            d_theta = d_theta - angle / curve
        end
        append!(theta,[theta[end]+d_theta])
        curv_curr = d_theta/ds
        append!(curvature,[curv_curr])
    end
    return theta,curvature
end

type Track
    xy::Array{Float64,2}
    idx     # not dummy idx vector, which will be useful when finding the nearest point in function "find_idx()"
    bound1xy::Array{Float64,2} # bound data are only for plotting visualization
    bound2xy::Array{Float64,2} # bound data are only for plotting visualization
    curvature::Array{Float64,1}
    max_curvature::Float64
    theta::Array{Float64,1}
    ds::Float64 # track discretization distance
    n_node::Int64 # number of points in the track
    w::Float64  # track width
    s::Float64  # track total length
    function Track(track_data::Array{Float64,2})
        # object Initialization
        track=new()

        xy=[0.0 0.0] # 1.x 2.y
        theta=[0.0]
        # theta=[pi/4]
        curvature=[0.0]
        ds=0.03 # length of each segment
        width=0.8
        # bound1xy=[0.0 width/2]
        # bound2xy=[0.0 -width/2]
        bound1xy = xy + width/2*[cos(theta[1]+pi/2) sin(theta[1]+pi/2)]
        bound2xy = xy - width/2*[cos(theta[1]+pi/2) sin(theta[1]+pi/2)]

        # create the track segment angle
        for i=1:size(track_data,1)
            (theta,curvature)=add_curve(theta,curvature,track_data[i,1],track_data[i,2],ds)
        end
        max_curvature=maximum(abs(curvature))
        # create the track segment by segment using the angle just calculated
        for i=2:size(theta,1) # we will have one more point at the end of the array, which is the same as the original starting point
            xy_next=[xy[end,1]+ds*cos(theta[i]) xy[end,2]+ds*sin(theta[i])]
            bound1xy_next = xy_next + width/2*[cos(theta[i]+pi/2) sin(theta[i]+pi/2)]
            bound2xy_next = xy_next - width/2*[cos(theta[i]+pi/2) sin(theta[i]+pi/2)]
            xy=vcat(xy,xy_next)
            bound1xy=vcat(bound1xy,bound1xy_next)
            bound2xy=vcat(bound2xy,bound2xy_next)
        end

        # object construction
        track.xy=xy; track.bound1xy=bound1xy; track.bound2xy=bound2xy;
        track.curvature=curvature; track.max_curvature=max_curvature;
        track.theta=theta; track.ds=ds; track.w=width
        track.n_node=size(xy,1); track.idx=1:size(xy,1)
        track.s=(track.n_node-1)*track.ds # Important: exclude the last point from the track length!
        return track
    end
    # The last point of this function is actually a dummy point
end


