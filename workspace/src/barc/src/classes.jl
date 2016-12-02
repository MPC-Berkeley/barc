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
    coeffCost::Array{Float64}
    coeffConst::Array{Float64}
    order::Int64
    pLength::Int64      # small values here may lead to numerical problems since the functions are only approximated in a short horizon
                        # "small" values are about 2*N, good values about 4*N
                        # numerical problems occur at the edges (s=0, when v is almost 0 and s does not change fast and at s=s_target)
    c_Vx::Array{Float64,1}
    c_Vy::Array{Float64,1}
    c_Psi::Array{Float64,1}
    MpcCoeff(coeffCost=Float64[], coeffConst=Float64[], order=4, pLength=0,c_Vx=Float64[],c_Vy=Float64[],c_Psi=Float64[]) = new(coeffCost, coeffConst, order, pLength, c_Vx, c_Vy, c_Psi)
end

type OldTrajectory      # information about previous trajectories
    oldTraj::Array{Float64}             # contains all states over all laps
    oldInput::Array{Float64}            # contains all inputs
    oldTimes::Array{Float64}            # contains times related to states and inputs
    oldCost::Array{Int64}               # contains costs of laps
    count::Array{Int64}                 # contains the counter for each lap
    prebuf::Int64
    postbuf::Int64
    idx_start::Array{Int64}             # index of the first measurement with s > 0
    idx_end::Array{Int64}               # index of the last measurement with s < s_target
    OldTrajectory(oldTraj=Float64[],oldInput=Float64[],oldTimes=Float64[],oldCost=Float64[],count=Int64[],prebuf=50,postbuf=50,idx_start=Int64[],idx_end=Int64[]) = new(oldTraj,oldInput,oldTimes,oldCost,count,prebuf,postbuf,idx_start,idx_end)
end

type MpcParams          # parameters for MPC solver
    N::Int64
    nz::Int64
    OrderCostCons::Int64
    Q::Array{Float64,1}
    Q_term::Array{Float64,1}
    R::Array{Float64,1}
    vPathFollowing::Float64
    QderivZ::Array{Float64,1}
    QderivU::Array{Float64,1}
    Q_term_cost::Float64
    delay_df::Int64
    delay_a::Int64
    MpcParams(N=0,nz=0,OrderCostCons=0,Q=Float64[],Q_term=Float64[],R=Float64[],vPathFollowing=1.0,QderivZ=Float64[],QderivU=Float64[],Q_term_cost=1.0,delay_df=0,delay_a=0) = new(N,nz,OrderCostCons,Q,Q_term,R,vPathFollowing,QderivZ,QderivU,Q_term_cost,delay_df,delay_a)
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
    MpcSol(a_x=0.0,d_f=0.0,solverStatus=Symbol(),u=Float64[],z=Float64[],cost=Float64[]) = new(a_x,d_f,solverStatus,u,z,cost)
end

type TrackCoeff         # coefficients of track
    coeffAngle::Array{Float64,1}
    coeffCurvature::Array{Float64,1}
    nPolyCurvature::Int64      # order of the interpolation polynom
    width::Float64               # lane width -> is used in cost function as soft constraints (to stay on track)
    TrackCoeff(coeffAngle=Float64[],coeffCurvature=Float64[],nPolyCurvature=4,width=1.0) = new(coeffAngle,coeffCurvature,nPolyCurvature,width)
end

type ModelParams
    l_A::Float64
    l_B::Float64
    m::Float64
    I_z::Float64
    dt::Float64
    u_lb::Array{Float64}        # lower bounds for u
    u_ub::Array{Float64}        # upper bounds
    z_lb::Array{Float64}
    z_ub::Array{Float64}
    c0::Array{Float64}
    c_f::Float64
    ModelParams(l_A=0.25,l_B=0.25,m=1.98,I_z=0.24,dt=0.1,u_lb=Float64[],u_ub=Float64[],z_lb=Float64[],z_ub=Float64[],c0=Float64[],c_f=0.0) = new(l_A,l_B,m,I_z,dt,u_lb,u_ub,z_lb,z_ub,c0,c_f)
end
