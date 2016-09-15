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
    coeffCost
    coeffConst
    order
    pLength             # small values here may lead to numerical problems since the functions are only approximated in a short horizon
                        # "small" values are about 2*N, good values about 4*N
                        # numerical problems occur at the edges (s=0, when v is almost 0 and s does not change fast and at s=s_target)
    MpcCoeff(coeffCost=0, coeffConst=0, order=4, pLength=0) = new(coeffCost, coeffConst, order, pLength)
end

type OldTrajectory      # information about previous trajectories
    oldTraj
    oldInput
    oldCost
    OldTrajectory(oldTraj=0,oldInput=0,oldCost=0) = new(oldTraj,oldInput,oldCost)
end

type MpcParams          # parameters for MPC solver
    N::Int64
    nz::Int64
    OrderCostCons::Int64
    Q
    R
    vPathFollowing::Float64
    QderivZ
    QderivU
    MpcParams(N=0,nz=0,OrderCostCons=0,Q=0,R=0,vPathFollowing=1.0,QderivZ=0.0,QderivU=0.0) = new(N,nz,OrderCostCons,Q,R,vPathFollowing)
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
    solverStatus
    u
    z
    cost
    MpcSol(a_x=0,d_f=0,solverStatus=0,u=0,z=0,cost=0) = new(a_x,d_f,solverStatus,u,z,cost)
end

type TrackCoeff         # coefficients of track
    coeffAngle
    coeffCurvature
    nPolyCurvature      # order of the interpolation polynom
    width               # lane width -> is used in cost function as soft constraints (to stay on track)
    TrackCoeff(coeffAngle=0,coeffCurvature=0,nPolyCurvature=4) = new(coeffAngle,coeffCurvature,nPolyCurvature)
end

type ModelParams
    l_A::Float64
    l_B::Float64
    dt::Float64
    u_lb        # lower bounds for u
    u_ub        # upper bounds
    z_lb
    z_ub
    c0
    ModelParams(l_A=0.25,l_B=0.25,dt=0.1,u_lb=0,u_ub=0,z_lb=0,z_ub=0,c0=0) = new(l_A,l_B,dt,u_lb,u_ub,z_lb,z_ub,c0)
end