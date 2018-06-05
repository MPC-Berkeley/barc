# VARIOUS TYPES FOR CALCULATIONS

type LapStatus
    currentLap::Int64       # current lap number
    currentIt::Int64        # current iteration in current lap
    switchLap::Bool
    nextLap::Bool
    s_lapTrigger::Float64
end

type MpcCoeff           # coefficients for trajectory approximation
    c_Vx::Array{Float64,2}
    c_Vy::Array{Float64,2}
    c_Psi::Array{Float64,2}
    MpcCoeff(c_Vx=zeros(10,6),c_Vy=zeros(10,4),c_Psi=zeros(10,3)) = new(c_Vx, c_Vy, c_Psi)
    # c_Vx::Array{Float64,2}
    # c_Vy::Array{Float64,1}
    # c_Psi::Array{Float64,1}
    # MpcCoeff(c_Vx=zeros(10,3),c_Vy=zeros(10),c_Psi=zeros(10)) = new(c_Vx, c_Vy, c_Psi)
end

type SolHistory
    u::Array{Float64,4}
    z::Array{Float64,4}
    cost::Array{Float64,1}
    function SolHistory(bufferSize::Int64,N::Int64,n_state::Int64,n_lap::Int64)
        solHistory=new()
        solHistory.u=zeros(bufferSize,n_lap,N,2)
        solHistory.z=zeros(bufferSize,n_lap,N+1,n_state)
        solHistory.cost=2*ones(n_lap)
        return solHistory
    end
end

type SafeSet
    oldSS::Array{Float64}
    oldCost::Array{Int64}
    Np::Int64
    Nl::Int64
    selStates::Array{Float64,2}
    statesCost::Array{Float64,1}
    function SafeSet(BUFFERSIZE::Int64,lapNum::Int64)
        SS = new()
        n_state     = get_param("controller/n_state")
        SS.oldSS    = NaN*ones(BUFFERSIZE,n_state,lapNum)
        SS.oldCost  = ones(num_lap)
        SS.Np   = get_param("controller/Np")
        SS.Nl   = get_param("controller/Nl")
        SS.selStates = zeros(sel.Nl*sel.Np,n_state)
        SS.stateCost = zeros(sel.Nl*sel.Np)
        return SS
    end
end

type SysID
    feature_Np::Int64
    feature_Nl::Int64
    feature_z::Int64
    feature_u::Int64
    function SelectedStates()
        sysID = new()        
        n_state          = get_param("controller/n_state")
        sysID.feature_Np = get_param("controller/feature_Np")
        sysID.feature_Nl = get_param("controller/feature_Nl")
        sysID.feature_z = zeros(sysID.feature_Np*sysID.feature_Nl,n_state)
        sysID.feature_u = zeros(sysID.feature_Np*sysID.feature_Nl,n_state)
        return selectedStates
    end
end

type MpcParams
    # parameters for both PF and LMPC
    N::Int64
    # parameters for PF
    Q::Array{Float64,1}
    vPathFollowing::Float64
    # parameters for LMPC
    R::Array{Float64,1}
    QderivZ::Array{Float64,1}
    QderivU::Array{Float64,1}
    Q_term::Float64
    Q_lane::Float64
    Q_slack::Array{Float64,1}
    function MpcParams()
        mpcParams = new()
        mpcParams.N = get_param("controller/N")
        return mpcParams
    end
end

type MpcSol             # MPC solution output
    u::Array{Float64,2}
    z::Array{Float64,2}
    u_prev::Array{Float64,2}
    z_prev::Array{Float64,2}
    GP_e_vy::Array{Float64,1}
    GP_e_psiDot::Array{Float64,1}
    a_x::Float64
    d_f::Float64
    df_his::Array{Float64,1}
    a_his::Array{Float64,1}
    function MpcSol()
        mpcSol  = new()
        N       = get_param("controller/N")
        n_state = get_param("controller/n_state")
        mpcSol.u        = zeros(N,2)
        mpcSol.z        = zeros(N+1,n_state)
        mpcSol.u_prev   = zeros(N,2)
        mpcSol.z_prev   = zeros(N+1,n_state)
        mpcSol.GP_e_vy  = zeros(N)
        mpcSol.GP_e_psiDot = zeros(N)
        mpcSol.a_x      = 0.0
        mpcSol.d_f      = 0.0
        mpcSol.a_his    = zeros(get_param("controller/delay_a"))
        mpcSol.df_his   = zeros(get_param("controller/delay_a"))
        return mpcSol
    end
end

type PosInfo
    s::Float64
    ey::Float64
    epsi::Float64
    v::Float64
    x::Float64
    y::Float64
    vx::Float64
    vy::Float64
    psi::Float64
    psiDot::Float64
    ax::Float64
    ay::Float64
    a::Float64
    df::Float64
    function PosInfo()
        posInfo = new()
        posInfo.s       = 0.0
        posInfo.ey      = 0.0
        posInfo.epsi    = 0.0
        posInfo.v       = 0.0
        posInfo.x       = 0.0
        posInfo.y       = 0.0
        posInfo.vx      = 0.0
        posInfo.vy      = 0.0
        posInfo.psi     = 0.0
        posInfo.psiDot  = 0.0
        posInfo.ax      = 0.0
        posInfo.ay      = 0.0
        posInfo.a       = 0.0
        posInfo.df      = 0.0
        return posInfo
    end
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
        if get_param("feature_flag")
            theta=[pi/4]
        else
            theta=[0.0]
        end
            
        curvature=[0.0]
        ds=0.01 # length of each segment
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
end


