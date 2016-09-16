# VARIOUS TYPES FOR CALCULATIONS

type LapStatus
    currentLap::Int64       # current lap number
    currentIt::Int64        # current iteration in current lap
end

# Structure of coeffConst:
# 1st dimension specifies the state (1 = eY, 2 = ePsi, 3 = v)
# 2nd dimension is the polynomial coefficient
# 3rd dimension is not used
# 4th dimension specifies one of the two lap numbers between which are iterated

type MpcCoeff           # coefficients for trajectory approximation
    coeffCost::Array{Float64}
    coeffConst::Array{Float64}
    order::Int64
    pLength::Int64      # small values here may lead to numerical problems since the functions are only approximated in a short horizon
                        # "small" values are about 2*N, good values about 4*N
                        # numerical problems occur at the edges (s=0, when v is almost 0 and s does not change fast and at s=s_target)
    MpcCoeff(coeffCost=Float64[], coeffConst=Float64[], order=4, pLength=0) = new(coeffCost, coeffConst, order, pLength)
end

type OldTrajectory      # information about previous trajectories
    oldTraj::Array{Float64}
    oldInput::Array{Float64}
    oldCost::Array{Int64}
    OldTrajectory(oldTraj=Float64[],oldInput=Float64[],oldCost=Float64[]) = new(oldTraj,oldInput,oldCost)
end

type MpcParams          # parameters for MPC solver
    N::Int64
    nz::Int64
    OrderCostCons::Int64
    Q::Array{Float64,1}
    R::Array{Float64,1}
    vPathFollowing::Float64
    QderivZ::Array{Float64}
    QderivU::Array{Float64}
    MpcParams(N=0,nz=0,OrderCostCons=0,Q=Float64[],R=Float64[],vPathFollowing=1.0,QderivZ=Float64[],QderivU=Float64[]) = new(N,nz,OrderCostCons,Q,R,vPathFollowing)
end

type PosInfo            # current position information
    s_start::Float64
    s::Float64
    s_target::Float64
    PosInfo(s_start=0,s=0,s_target=0) = new(s_start,s,s_target)
end

type MpcSol             # MPC solution output
    a_x::Float64
    d_f::Float64
    solverStatus::Int64
    u::Array{Float64}
    z::Array{Float64}
    cost::Array{Float64}
    MpcSol(a_x=0.0,d_f=0.0,solverStatus=1,u=Float64[],z=Float64[],cost=Float64[]) = new(a_x,d_f,solverStatus,u,z,cost)
end

type TrackCoeff         # coefficients of track
    coeffAngle::Array{Float64}
    coeffCurvature::Array{Float64}
    nPolyCurvature::Int64      # order of the interpolation polynom
    width::Float64               # lane width -> is used in cost function as soft constraints (to stay on track)
    TrackCoeff(coeffAngle=Float64[],coeffCurvature=Float64[],nPolyCurvature=4,width=1.0) = new(coeffAngle,coeffCurvature,nPolyCurvature)
end

type ModelParams
    l_A::Float64
    l_B::Float64
    dt::Float64
    u_lb::Array{Float64}        # lower bounds for u
    u_ub::Array{Float64}        # upper bounds
    z_lb::Array{Float64}
    z_ub::Array{Float64}
    c0::Array{Float64}
    ModelParams(l_A=0.25,l_B=0.25,dt=0.1,u_lb=Float64[],u_ub=Float64[],z_lb=Float64[],z_ub=Float64[],c0=Float64[]) = new(l_A,l_B,dt,u_lb,u_ub,z_lb,z_ub,c0)
end