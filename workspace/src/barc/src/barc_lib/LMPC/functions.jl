# Important information about oldTraj:
# ======================================
# oldTraj.oldTraj contains all state information of one lap. It is structured like follows:
# The first <prebuf> values are the end of one lap before the next lap starts (necessary for inter-lap-system-ID)
# The last value of <prebuf> ends with the last value *before* the finish line (s < s_target)
# The s-values within <prebuf> need to be below zero (-> before the finish line). Otherwise they can be mistaken as the end of the current (not previous) lap.
# After <prebuf> follow the recorded states of the trajectory (as many as there were during one lap)
# The number of recorded states during one trajectory (0 <= s < s_target) is equal to <oldCost>.
# After the recorded trajectory, the rest of the vector (until <buffersize>) is filled up with constant values

function saveOldTraj(oldTraj::OldTrajectory,zCurr::Array{Float64},uCurr::Array{Float64},lapStatus::LapStatus,posInfo::PosInfo,buffersize::Int64)
    println("Starting function")
    i               = lapStatus.currentIt-1         # i = number of points for 0 <= s < s_target (= cost of this lap)
    prebuf          = oldTraj.prebuf                # so many points of the end of the previous old traj will be attached to the beginning
    zCurr_export    = zeros(buffersize,6)
    uCurr_export    = zeros(buffersize,2)

    println("Creating exports")
    zCurr_export    = cat(1,oldTraj.oldTraj[oldTraj.oldCost[1]+1:oldTraj.oldCost[1]+prebuf,:,1],
                            zCurr[1:i,:], NaN*ones(buffersize-i-prebuf,6))#[ones(buffersize-i-prebuf,1)*zCurr[i,1:5] zCurr[i,6]+collect(1:buffersize-i-prebuf)*dt*zCurr[i,1]])
    uCurr_export    = cat(1,oldTraj.oldInput[oldTraj.oldCost[1]+1:oldTraj.oldCost[1]+prebuf,:,1],
                            uCurr[1:i,:], NaN*ones(buffersize-i-prebuf,2))#zeros(buffersize-i-prebuf,2))

    zCurr_export[1:prebuf,6] -= posInfo.s_target       # make the prebuf-values below zero
    costLap                   = i                      # the cost of the current lap is the time it took to reach the finish line
    println("Saving")
    # Save all data in oldTrajectory:
    if lapStatus.currentLap <= 2                        # if it's the first or second lap
        oldTraj.oldTraj[:,:,1]  = zCurr_export          # ... just save everything
        oldTraj.oldInput[:,:,1] = uCurr_export
        oldTraj.oldTraj[:,:,2]  = zCurr_export  
        oldTraj.oldInput[:,:,2] = uCurr_export
        oldTraj.oldCost = [costLap,costLap]
    else                                                # idea: always copy the new trajectory in the first array!
        if oldTraj.oldCost[1] < oldTraj.oldCost[2]      # if the first old traj is better than the second
            oldTraj.oldTraj[:,:,2]  = oldTraj.oldTraj[:,:,1]    # ... copy the first in the second
            oldTraj.oldInput[:,:,2] = oldTraj.oldInput[:,:,1]   # ... same for the input
            oldTraj.oldCost[2] = oldTraj.oldCost[1]
        end
        oldTraj.oldTraj[:,:,1]  = zCurr_export                 # ... and write the new traj in the first
        oldTraj.oldInput[:,:,1] = uCurr_export
        oldTraj.oldCost[1] = costLap
    end
end

# FUNCTIONS FOR LOCALIZATION
function find_idx(z::Array{Float64,2},track::Track)
    idx = Int(ceil(z[1]/track.ds)+1)
    idx>track.n_node ? idx=idx%track.n_node : nothing
    idx<0 ? idx+=track.n_node : nothing
    return idx
end

function curvature_prediction(z::Array{Float64,2},track::Track)
    if size(z,1)==1
        curvature=track.curvature[find_idx(z,track)]
    else
        curvature=zeros(size(z,1))
        for i=1:size(z,1)
            curvature[i]=track.curvature[find_idx(z[i,:],track)]
        end
    end
    return curvature
end

function trackFrame_to_xyFrame(z_sol::Array{Float64,2},track::Track)
    # Position sanity check
    n = size(z_sol,1)
    z_x = zeros(n)
    z_y = zeros(n)
    for i in 1:n
        z = z_sol[i,:]
        if z[1]>track.s
            z[1]-=track.s
        elseif z[1]<0
            z[1]+=track.s
        end # At the end of one lap, the prediction can be out of the lap idx range

        ds=track.ds; s=z[1]; ey=z[2]; epsi=z[3]
        idx=Int64(ceil(s/ds))+1 # correction for the starting original point
        idx>track.n_node ? idx=idx%track.n_node : nothing
        # println(idx)
        # println(track.n_node)

        x_track=track.xy[idx,1]; y_track=track.xy[idx,2]; theta=track.theta[idx]
        x=x_track+ey*cos(theta+pi/2); y=y_track+ey*sin(theta+pi/2)
        # psi=theta+epsi
        z_x[i]=x
        z_y[i]=y
    end
    return z_x, z_y
end

# FUNCTIONS FOR FEATURE DATA SELECTING AND SYS_ID
function find_feature_dist(z_feature::Array{Float64,3},u_feature::Array{Float64,2},z_curr::Array{Float64,2},u_curr::Array{Float64,2},)
    Np=40 # Just to make life easier, we directly put a specific number here
    # Clear the safe set data of previous iteration
    iden_z=zeros(Np,3,2)
    iden_u=zeros(Np,2)

    # curr_state=hcat(z_curr[1],z_curr[4:6]',u_curr')
    # norm_state=[0.5 1 0.1 1 1 0.3] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
    curr_state=hcat(z_curr[4:6]',u_curr)

    norm_state=[1 0.1 1 1/2 1/2] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
    dummy_state=z_feature[:,:,1]
    dummy_input=u_feature
    # cal_state=Float64[] # stored for normalization calculation
    cal_state=hcat(dummy_state,dummy_input)
    dummy_norm=zeros(size(dummy_state,1),2)

    norm_dist=(curr_state.-cal_state[:,vcat(4:8),1])./norm_state
    dummy_norm[:,1]=norm_dist[:,1].^2+norm_dist[:,4].^2+norm_dist[:,5].^2+norm_dist[:,5].^2 #+norm_dist[:,5].^2
    dummy_norm[:,2]=1:size(dummy_state,1)
    dummy_norm=sortrows(dummy_norm) # pick up the first minimum Np points
    # println(dummy_norm[1:Np,:])
    for i=1:Np
        iden_z[i,:,1]=z_feature[Int(dummy_norm[i,2]),4:6,1]
        iden_z[i,:,2]=z_feature[Int(dummy_norm[i,2]),4:6,2]
        # iden_z_plot[i,:]=dummy_state[Int(dummy_norm[i,2]),:,1]
        iden_u[i,:]=dummy_input[Int(dummy_norm[i,2]),:]
    end

    # iden_z: Npx3x2 states selected for system identification
    # iden_u: Npx2 inputs selected for system identification
    return iden_z, iden_u
end

function coeff_iden_dist(idenStates::Array{Float64,3},idenInputs::Array{Float64,2})
    z = idenStates
    u = idenInputs
    # println(z)
    # println(u)
    size(z,1)==size(u,1) ? nothing : error("state and input in coeff_iden() need to have the same dimensions")
    A_vx=zeros(size(z,1),3)
    A_vy=zeros(size(z,1),4)
    A_psi=zeros(size(z,1),3)
    # y_vx=diff(z[:,1,:],2)
    # y_vy=diff(z[:,2,:],2)
    # y_psi=diff(z[:,3,:],2)
    y_vx=diff(reshape(z[:,1,:],size(z,1),size(z,3)),2)
    y_vy=diff(reshape(z[:,2,:],size(z,1),size(z,3)),2)
    y_psi=diff(reshape(z[:,3,:],size(z,1),size(z,3)),2)
    for i=1:size(z,1)
        A_vx[i,1]=z[i,2,1]*z[i,3,1]
        A_vx[i,2]=z[i,1,1]
        A_vx[i,3]=u[i,1]
        A_vy[i,1]=z[i,2,1]/z[i,1,1]
        A_vy[i,2]=z[i,1,1]*z[i,3,1]
        A_vy[i,3]=z[i,3,1]/z[i,1,1]
        A_vy[i,4]=u[i,2]
        A_psi[i,1]=z[i,3,1]/z[i,1,1]
        A_psi[i,2]=z[i,2,1]/z[i,1,1]
        A_psi[i,3]=u[i,2]
    end
    c_Vx = A_vx\y_vx
    c_Vy = A_vy\y_vy
    c_Psi = A_psi\y_psi
    # println("c_Vx is $c_Vx")
    # println("c_Vx is $c_Vy")
    # println("c_Vx is $c_Psi")
    return c_Vx, c_Vy, c_Psi
end

function find_SS(safeSetData::SafeSetData,selectedStates::SelectedStates,
                 z_prev::Array{Float64,2},lapStatus::LapStatus,
                 modelParams::ModelParams,mpcParams::MpcParams,track::Track)

    s=z_prev[2,1]; v=z_prev[2,4];
    N=mpcParams.N; dt=modelParams.dt
    Nl=selectedStates.Nl; Np_here=copy(selectedStates.Np/2)
    # Clear the safe set data of previous iteration
    selectedStates.selStates=Array{Float64}(0,6);
    selectedStates.statesCost=Array{Float64}(0,1);
    # target_s=s+v*dt*(N)   # 3 is a temporary shift number
    target_s=z_prev[end,1]+z_prev[end,4]*dt
    # for i=1:lapStatus.currentLap-1
    for i=1:Nl
        SS_curr=safeSetData.oldSS[:,:,lapStatus.currentLap-i]
        SScost_curr=copy(safeSetData.cost2target[:,lapStatus.currentLap-i])
        all_s=SS_curr[1:Int(safeSetData.oldCost[lapStatus.currentLap-i]),1]
        if target_s>track.s
            target_s-=track.s # correction when switching the lap
        end
        (value_s,idx_s)=findmin(abs(all_s-target_s))
        idx_s_start=Int(idx_s-Np_here);
        # idx_s_start=max(idx_s_start,find_idx(z_prev[2,:],track));
        idx_s_end=Int(idx_s+Np_here-1) # to be consistant with the number of points selected in the safe set
        # idx sanity check
        cost=Int(safeSetData.oldCost[lapStatus.currentLap-i])
        # println(safeSetData.oldCost)
        # println("Target s is: $target_s")
        # println("Safe set idx start and end are")
        # println(idx_s_start)
        # println(idx_s_end)
        # println("Size of SS_curr is")
        # println(size(SS_curr))
        # println("Size of SS_cost is")
        # println(size(SScost_curr))
        
        if idx_s_start<1
            SS_curr[1:idx_s_end,1]+=track.s # correction when switching lap
            selectedStates.selStates=vcat(selectedStates.selStates,SS_curr[cost+idx_s_start:cost,:],SS_curr[1:idx_s_end,:])
            SScost_curr[1:idx_s_end]-=cost # correction when switching lap
            selectedStates.statesCost=vcat(selectedStates.statesCost,SScost_curr[cost+idx_s_start:cost],SScost_curr[1:idx_s_end])
        elseif idx_s_end>cost
            SS_curr[1:idx_s_end-cost,1]+=track.s
            selectedStates.selStates=vcat(selectedStates.selStates,SS_curr[idx_s_start:cost,:],SS_curr[1:idx_s_end-cost,:])
            SScost_curr[1:idx_s_end-cost]-=cost
            selectedStates.statesCost=vcat(selectedStates.statesCost,SScost_curr[idx_s_start:cost],SScost_curr[1:idx_s_end-cost])
        else
            if s>track.s-v*dt*(N)
                SS_curr[idx_s_start:idx_s_end,1]+=track.s
                selectedStates.selStates=vcat(selectedStates.selStates,SS_curr[idx_s_start:idx_s_end,:])
                selectedStates.statesCost=vcat(selectedStates.statesCost,SScost_curr[idx_s_start:idx_s_end])
            else
                # println("idx_s_start")
                # println(idx_s_start)
                # println("idx_s_end")
                # println(idx_s_end)
                selectedStates.selStates=vcat(selectedStates.selStates,SS_curr[idx_s_start:idx_s_end,:])
                selectedStates.statesCost=vcat(selectedStates.statesCost,SScost_curr[idx_s_start:idx_s_end])
                # selectedStates.selStates=SS_curr[idx_s_start:idx_s_end,:]
                # selectedStates.statesCost=SScost_curr[idx_s_start:idx_s_end]
            end
        end
    end
    return selectedStates
end

# FUNCTION FOR PARAMETERS INITIALIZATION
function InitializeParameters(mpcParams::MpcParams,mpcParams_pF::MpcParams,modelParams::ModelParams,
                              posInfo::PosInfo,oldTraj::OldTrajectory,mpcCoeff::MpcCoeff,lapStatus::LapStatus,buffersize::Int64,
                              selectedStates::SelectedStates,oldSS::SafeSetData)

    simulator_flag = true     # set this to TRUE if SIMULATOR is in use, set this to FALSE if BARC is in use

    if simulator_flag == false   # if the BARC is in use

        # selectedStates.Nl_sID       = 3
        # selectedStates.lambda1      = 0
        # selectedStates.lambda2      = 0
        # selectedStates.lambda3      = 0
        selectedStates.Np           = 20                           # Number of points to take from each previous trajectory to build the convex hull
        selectedStates.Nl           = 2                             # Number of previous laps to include in the convex hull
        # selectedStates.shift        = 8
        Nl                          = selectedStates.Nl
        selectedStates.selStates    = zeros(Nl*selectedStates.Np,6)  
        selectedStates.statesCost   = zeros(Nl*selectedStates.Np)
        # selectedStates.version      = false

        mpcParams.N                 = 10
        mpcParams.Q                 = [5.0,0.0,0.0,0.1,50.0,0.0]   # Q (only for path following mode)
        mpcParams.vPathFollowing    = 1                           # reference speed for first lap of path following
        mpcParams.Q_term            = 1.0*[20.0,1.0,10.0,20.0,50.0]   # weights for terminal constraints (LMPC, for xDot,yDot,psiDot,ePsi,eY).Not used if using convex hull
        mpcParams.R                 = 0*[10.0,10.0]                 # put weights on a and d_f
        mpcParams.QderivZ           = 10.0*[1,1,1,1,1,1]             # cost matrix for derivative cost of states
        mpcParams.QderivU           = 1.0*[5.0,0.1] #NOTE Set this to [5.0, 0/40.0]              # cost matrix for derivative cost of inputs
        mpcParams.Q_term_cost       = 5                        # scaling of Q-function
        mpcParams.delay_df          = 3                             # steering delay
        mpcParams.delay_a           = 1                             # acceleration delay
        mpcParams.Q_lane            = 1                      # weight on the soft constraint for the lane
        mpcParams.Q_vel             = 1                    # weight on the soft constraint for the maximum velocity
        mpcParams.Q_slack           = 1*[5*20.0,0.5*20.0,1*10.0,30.0,0.1*80.0,50.0]#[20.0,10.0,10.0,30.0,80.0,50.0]  #vx,vy,psiDot,ePsi,eY,s
        mpcParams.Q_obs = ones(Nl*selectedStates.Np)# weight to esclude some of the old trajectories
    
    elseif simulator_flag == true  # if the simulator is in use

        selectedStates.Np           = 10        # please select an even number
        selectedStates.Nl           = 2         # Number of previous laps to include in the convex hull
        # selectedStates.Nl_sID       = 3
        # selectedStates.lambda1      = 1
        # selectedStates.lambda2      = 1
        # selectedStates.lambda3      = 1
        # selectedStates.shift        = 8
        Nl                          = selectedStates.Nl
        selectedStates.selStates    = zeros(Nl*selectedStates.Np,6)  
        selectedStates.statesCost   = zeros(Nl*selectedStates.Np)
        # selectedStates.version      = false

        mpcParams.N                 = 10
        mpcParams.Q                 = [5.0,0.0,0.0,0.1,50.0,0.0]   # Q (only for path following mode)
        mpcParams.vPathFollowing    = 1                           # reference speed for first lap of path following
        mpcParams.Q_term            = 1.0*[20.0,1.0,10.0,20.0,50.0]   # weights for terminal constraints (LMPC, for xDot,yDot,psiDot,ePsi,eY).Not used if using convex hull
        mpcParams.R                 = 0*[10.0,10.0]                 # put weights on a and d_f
        mpcParams.QderivZ           = 10.0*[1,1,1,1,1,1]             # cost matrix for derivative cost of states
        mpcParams.QderivU           = 1.0*[1.0,1.0] #NOTE Set this to [5.0, 0/40.0]              # cost matrix for derivative cost of inputs
        mpcParams.Q_term_cost       = 0.5                        # scaling of Q-function
        mpcParams.delay_df          = 3                             # steering delay
        mpcParams.delay_a           = 1                             # acceleration delay
        mpcParams.Q_lane            = 10                      # weight on the soft constraint for the lane
        mpcParams.Q_vel             = 1                    # weight on the soft constraint for the maximum velocity
        # mpcParams.Q_slack           = 1*[20.0,1.0,10.0,30.0,80.0,50.0]#[20.0,10.0,10.0,30.0,80.0,50.0]  #vx,vy,psiDot,ePsi,eY,s
        mpcParams.Q_slack           = 1.0*[50.0,80.0,30.0,20.0,1.0,10.0]#[20.0,10.0,10.0,30.0,80.0,50.0]  #vx,vy,psiDot,ePsi,eY,s
        mpcParams.Q_obs             = ones(Nl*selectedStates.Np)# weight to esclude some of the old trajectories    
    end

    mpcParams_pF.N              = 10
    mpcParams_pF.Q              = [0.0,20.0,10.0,10.0]
    mpcParams_pF.R              = 0*[1.0,1.0]               # put weights on a and d_f
    mpcParams_pF.QderivZ        = 0.0*[0.0,0,1.0,0]           # cost matrix for derivative cost of states
    mpcParams_pF.QderivU        = 1*[1,1]                # cost matrix for derivative cost of inputs
    mpcParams_pF.vPathFollowing = 1                       # reference speed for first lap of path following
    mpcParams_pF.delay_df       = 3                         # steering delay (number of steps)
    mpcParams_pF.delay_a        = 1                         # acceleration delay

    # trackCoeff.nPolyCurvature   = 8                         # 4th order polynomial for curvature approximation
    # trackCoeff.coeffCurvature   = zeros(trackCoeff.nPolyCurvature+1)         # polynomial coefficients for curvature approximation (zeros for straight line)
    # trackCoeff.width            = 0.6                       # width of the track (60cm)

    modelParams.l_A             = 0.125
    modelParams.l_B             = 0.125
    modelParams.dt              = 0.1                   # sampling time, also controls the control loop, affects delay_df and Qderiv
    modelParams.m               = 1.98
    modelParams.I_z             = 0.03
    modelParams.c_f             = 0.5                   # friction coefficient: xDot = - c_f*xDot (aerodynamic+tire)

    # posInfo.s_target            = 19.11
    num_lap = 100
    oldTraj.oldTraj             = NaN*ones(buffersize,7,num_lap)
    oldTraj.oldInput            = zeros(buffersize,2,num_lap)
    oldTraj.oldTimes            = NaN*ones(buffersize,num_lap)
    oldTraj.count               = ones(num_lap)*2
    oldTraj.oldCost             = ones(Int64,num_lap)                   # dummies for initialization
    # oldTraj.prebuf              = 30
    # oldTraj.postbuf             = 30
    oldTraj.idx_start           = zeros(num_lap)
    oldTraj.idx_end             = zeros(num_lap)

    oldSS.oldSS                 = NaN*ones(buffersize,6,num_lap)          # contains data from previous laps usefull to build the safe set
    oldSS.oldSS_xy              = NaN*ones(buffersize,4,num_lap)          
    oldSS.cost2target           = zeros(buffersize,num_lap)     # cost to arrive at the target, i.e. how many iterations from the start to the end of the lap
    oldSS.oldCost               = ones(Int64,num_lap)              # contains costs of laps
    oldSS.count                 = ones(num_lap)*2               # contains the counter for each lap
    # oldSS.prebuff                = 30
    # oldSS.postbuff               = 50
    oldSS.idx_start             = ones(num_lap)            # index of the first measurement with s > 0
    oldSS.idx_end               = zeros(num_lap)              # index of the last measurement with s < s_target

    # mpcCoeff.order              = 5
    # mpcCoeff.coeffCost          = zeros(mpcCoeff.order+1,2)
    # mpcCoeff.coeffConst         = zeros(mpcCoeff.order+1,2,5)
    # mpcCoeff.pLength            = 5*2*mpcParams.N        # small values here may lead to numerical problems since the functions are only approximated in a short horizon
    mpcCoeff.c_Vx               = zeros(mpcParams.N,3)
    mpcCoeff.c_Vy               = zeros(mpcParams.N,4)
    mpcCoeff.c_Psi              = zeros(mpcParams.N,3)

    lapStatus.currentLap        = 1         # initialize lap number
    lapStatus.currentIt         = 1         # current iteration in lap

end


function s6_to_s4(z::Array{Float64})
    if size(z,1)==1
        z=vcat(z[1:3],sqrt(z[4]^2+z[5]^2))
    elseif size(z,1)==2
        z=hcat(z[:,1:3],sqrt(z[:,4].^2+z[:,5].^2))
    else
        error("Please check the input dimension of function \"6s_to_4s\" ")
    end
    return z
end

# FUNCTIONS FOR MODEL SIM
function car_sim_kin(z::Array{Float64},u::Array{Float64},track::Track,modelParams::ModelParams)
    # Car parameters
    dt  = modelParams.dt
    L_a = modelParams.l_A
    L_b = modelParams.l_B

    idx=Int(ceil(z[1]/track.ds))+1 # correct the starting original point idx problem
    idx>track.n_node ? idx=idx%track.n_node : nothing
    idx<0 ? idx+=track.n_node : nothing

    # println(idx)
    # println(track.n_node)
    c=track.curvature[idx]

    bta = atan(L_a/(L_a+L_b)*tan(u[2]))
    dsdt = z[4]*cos(z[3]+bta)/(1-z[2]*c)

    zNext = copy(z)
    zNext[1] = z[1] + dt*dsdt                               # s
    zNext[2] = z[2] + dt*z[4] * sin(z[3] + bta)             # eY
    zNext[3] = z[3] + dt*(z[4]/L_b*sin(bta)-dsdt*c)         # ePsi
    zNext[4] = z[4] + dt*(u[1] - modelParams.c_f*z[4])      # v

    return zNext
end

function car_pre_dyn(z_curr::Array{Float64},u_sol::Array{Float64},track::Track,modelParams::ModelParams,outputDims::Int64)
    if size(u_sol,1)==1
        (z_dummy, Fy_dummy, a_slip_dummy)=car_sim_dyn_exact(z_curr,u_sol,track,modelParams)
        if outputDims==4
            z_dummy=s6_to_s4(z_dummy)
        end
    elseif size(u_sol,2)==2
        z_dummy=zeros(size(u_sol,1)+1,6)
        Fy_dummy=zeros(size(u_sol,1)+1,2)
        a_slip_dummy=zeros(size(u_sol,1)+1,2)

        z_dummy[1,:]=z_curr
        for i=2:size(u_sol,1)+1
            (z_dummy[i,:], Fy_dummy[i,:], a_slip_dummy[i,:])=car_sim_dyn_exact(z_dummy[i-1,:],u_sol[i-1,:],track,modelParams)
        end
        if outputDims==4
            z_dummy=s6_to_s4(z_dummy)
        end
    else
        error("Please check the u_sol dimension of function \"car_pre_dyn\" ")
    end

    return z_dummy, Fy_dummy, a_slip_dummy
end

function car_sim_dyn_exact(z::Array{Float64,2},u::Array{Float64,2},track::Track,modelParams::ModelParams)
    # This function uses smaller steps to achieve higher fidelity than we would achieve using longer timesteps
    z_final = copy(z)
    Fy=zeros(2); a_slip=zeros(2);
    u[1] = min(u[1],2)
    u[1] = max(u[1],-1)
    u[2] = min(u[2],0.1*pi)
    u[2] = max(u[2],-0.1*pi)
    dt=modelParams.dt; dtn=dt/10
    for i=1:10
        (z_final, Fy, a_slip)= car_sim_dyn(z_final,u,dtn,track,modelParams)
    end
    return z_final, Fy, a_slip
end

function car_sim_dyn(z::Array{Float64},u::Array{Float64},dt::Float64,track::Track,modelParams::ModelParams)
    # s, ey, epsi, vx, vy, Psidot
    L_f = modelParams.l_A
    L_r = modelParams.l_B
    m   = modelParams.m
    I_z = modelParams.I_z
    c_f = modelParams.c_f   # motor drag coefficient

    a_F = 0 # front tire slip angle
    a_R = 0 # rear tire slip angle
    if abs(z[4]) >= 0.1
        a_F     = atan((z[5] + L_f*z[6])/abs(z[4])) - u[2]
        a_R     = atan((z[5] - L_r*z[6])/abs(z[4]))
    else
        warn("too low speed, not able to simulate the dynamic model")
    end
    if max(abs(a_F),abs(a_R))>30/180*pi
        # warn("Large tire angles: a_F = $a_F, a_R = $a_R, xDot = $(z[4]), d_F = $(u[2])")
    end

    FyF = -pacejka(a_F)
    FyR = -pacejka(a_R)

    idx=Int(ceil(z[1]/track.ds))+1 # correct the starting original point idx problem
    # println(z[1])
    # println(idx)
    idx>track.n_node ? idx=idx%track.n_node : nothing
    idx<=0 ? idx += track.n_node : nothing
    # println(idx)
    
    # println(track.n_node)
    c=track.curvature[idx]
    

    dsdt = (z[4]*cos(z[3]) - z[5]*sin(z[3]))/(1-z[2]*c)

    zNext = copy(z)
    zNext[1] = z[1] + dt * dsdt                                             # s
    zNext[2] = z[2] + dt * (z[4]*sin(z[3]) + z[5]*cos(z[3]))                # eY
    zNext[3] = z[3] + dt * (z[6]-dsdt*c)                                    # ePsi
    zNext[4] = z[4] + dt * (u[1] + z[5]*z[6] - c_f*z[4])                    # vx
    zNext[5] = z[5] + dt * ((FyF*cos(u[2])+FyR)/m - z[4]*z[6])              # vy
    zNext[6] = z[6] + dt * ((L_f*FyF*cos(u[2]) - L_r*FyR)/I_z)              # psiDot

    return zNext, [FyF,FyR], [a_F,a_R]
end

function pacejka(a)
    # B = 1.0             # This value determines the steepness of the curve
    # C = 1.25
    B = 6.0             # This value determines the steepness of the curve
    C = 1.6
    mu = 0.8            # Friction coefficient (responsible for maximum lateral tire force)
    m = 1.98
    g = 9.81
    D = mu * m * g/2
    C_alpha_f = D*sin.(C*atan.(B*a))
    return C_alpha_f
end