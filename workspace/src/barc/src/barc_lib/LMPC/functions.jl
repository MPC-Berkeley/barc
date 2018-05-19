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
    idx<=0 ? idx+=track.n_node : nothing
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

function xyFrame_to_trackFrame(z::Array{Float64,1},track::Track) # Localization in Julia which is corresponding to Localization helper in python
    # x,y,vx,vy,psi,psidot -> s,ey,epsi,vx,vy,psidot
    dist = (z[1]-track.xy[:,1]).^2 + (z[2]-track.xy[:,2]).^2
    (val_min,idx_min)=findmin(dist)
    s = (idx_min-1)*track.ds
    theta = track.theta[idx_min]
    epsi = z[5] - theta
    # println(epsi)
    while epsi<-pi
        epsi += 2*pi
    end
    while epsi>pi
        epsi -= 2*pi
    end
    # epsi = epsi % pi
    # println(epsi)
    ey = sqrt(val_min)
    bound1_dist = (z[1]-track.bound1xy[idx_min,1]).^2 + (z[2]-track.bound1xy[idx_min,2]).^2
    bound2_dist = (z[1]-track.bound2xy[idx_min,1]).^2 + (z[2]-track.bound2xy[idx_min,2]).^2
    if bound1_dist > bound2_dist
        ey = -ey
    else
        ey = ey
    end
    return [s,ey,epsi,z[3],z[4],z[6]]
end

# FUNCTIONS FOR FEATURE DATA SELECTING AND SYS_ID
function find_feature_dist(z_feature::Array{Float64,3},u_feature::Array{Float64,2},z_curr::Array{Float64,2},u_curr::Array{Float64,2},selectedStates::SelectedStates)
    Np=selectedStates.feature_Np # Just to make life easier, we directly put a specific number here
    # Clear the safe set data of previous iteration
    iden_z=zeros(Np,3,2)
    iden_z_plot=zeros(Np,6)
    iden_u=zeros(Np,2)

    # curr_state=hcat(z_curr[1],z_curr[4:6]',u_curr')
    # norm_state=[0.5 1 0.1 1 1 0.3] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
    curr_state=hcat(z_curr[4:6]',u_curr)

    norm_state=[1 0.1 1 1 0.2] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
    # norm_state=[1 1 1 1/2 1/2] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
    dummy_state=z_feature[:,:,1]
    dummy_input=u_feature
    # cal_state=Float64[] # stored for normalization calculation
    cal_state=hcat(dummy_state,dummy_input)
    dummy_norm=zeros(size(dummy_state,1),2)

    norm_dist=(curr_state.-cal_state[:,4:8])./norm_state
    dummy_norm[:,1]=norm_dist[:,1].^2+norm_dist[:,2].^2+norm_dist[:,3].^2+norm_dist[:,4].^2+norm_dist[:,5].^2
    dummy_norm[:,2]=1:size(dummy_state,1)
    dummy_norm=sortrows(dummy_norm) # pick up the first minimum Np points
    # println(dummy_norm[1:Np,:])
    for i=1:Np
        iden_z[i,:,1]=z_feature[Int(dummy_norm[i,2]),4:6,1]
        iden_z[i,:,2]=z_feature[Int(dummy_norm[i,2]),4:6,2]
        iden_z_plot[i,:]=dummy_state[Int(dummy_norm[i,2]),:,1]
        iden_u[i,:]=dummy_input[Int(dummy_norm[i,2]),:]
    end

    # iden_z: Npx3x2 states selected for system identification
    # iden_u: Npx2 inputs selected for system identification
    return iden_z, iden_u, iden_z_plot
end

# THE DIFFERENCE TO THE FUNCTON ABOVE IS THAT THIS ONE IS SELECTING THE FEATURE POINTS FROM THE HISTORY
function find_SS_dist(solHistory::SolHistory,z_curr::Array{Float64,2},u_curr::Array{Float64,2},lapStatus::LapStatus,selectedStates::SelectedStates)

    Nl=selectedStates.feature_Nl
    Np=selectedStates.feature_Np
    # Clear the safe set data of previous iteration
    iden_z=zeros(Np,3,2)
    iden_z_plot=zeros(Np,6)
    iden_u=zeros(Np,2)

    # curr_state=hcat(z_curr[1],z_curr[4:6]',u_curr')
    # norm_state=[0.5 1 0.1 1 1 0.3] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
    curr_state=hcat(z_curr[4:6]',u_curr)
    norm_state=[1 0.1 1 1 0.2] # [1 0.1 1] are the normed state, the first state is for "s", which is for putting some weight on the track place
    dummy_state=Array{Float64}(0,6)
    dummy_input=Array{Float64}(0,2)
    cal_state=Array{Float64}(0,8) # stored for normalization calculation

    # collect out all the state and input history data
    # at maximum, Nl laps data will be collected, it was actually reshaped into the whole history
    for i=max(1,lapStatus.currentLap-Nl):max(1,(lapStatus.currentLap-1))
        dummy_state=vcat(dummy_state,reshape(solHistory.z[1:Int(solHistory.cost[i]),i,1,:],Int(solHistory.cost[i]),6))
        dummy_input=vcat(dummy_input,reshape(solHistory.u[1:Int(solHistory.cost[i]),i,1,:],Int(solHistory.cost[i]),2))
    end
    cal_state=vcat(cal_state,hcat(dummy_state,dummy_input))
    dummy_norm=zeros(size(dummy_state,1)-1,2)
    for i=1:size(dummy_state,1)-1 # The last state point will be removed
        # dummy_norm[i,1]=norm((curr_state-cal_state[i,vcat(1,4:8)]')./norm_state) # this is the state after normalization
        norm_dist=(curr_state.-cal_state[i,4:8])./norm_state
        dummy_norm[i,1]=norm_dist[1]^2+norm_dist[2]^2+norm_dist[3]^2+norm_dist[4]^2+norm_dist[5]^2
        # dummy_norm[i,1]=norm((curr_state-cal_state[i,4:8])./norm_state) # this is the state after normalization
        dummy_norm[i,2]=i
    end

    dummy_norm=sortrows(dummy_norm) # pick up the first minimum Np points

    for i=1:min(Np,size(dummy_norm,1))
        iden_z[i,:,1]=dummy_state[Int(dummy_norm[i,2]),4:6]
        iden_z[i,:,2]=dummy_state[Int(dummy_norm[i,2])+1,4:6]
        iden_z_plot[i,:]=dummy_state[Int(dummy_norm[i,2]),:]
        iden_u[i,:]=dummy_input[Int(dummy_norm[i,2]),:]
    end
    # iden_z: Npx3x2 states selected for system identification
    # iden_u: Npx2 inputs selected for system identification
    return iden_z, iden_u, iden_z_plot
end

function coeff_iden_dist(idenStates::Array{Float64,3},idenInputs::Array{Float64,2})
    z = idenStates
    # z[:,2,:]*=10
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

    # n = 0.02*randn(size(z,1),1)
    # n = max(n,-0.1)
    # n = min(n, 0.1)
    # z[:,1,1]+=n

    # n = 0.005*randn(size(z,1),1)
    # n = max(n,-0.005)
    # n = min(n, 0.005)
    # z[:,2,1]+=n

    # n = 0.05*randn(size(z,1),1)
    # n = max(n,-0.05)
    # n = min(n, 0.05)
    # z[:,3,1]+=n

    # n = 0.05*randn(size(z,1),1)
    # n = max(n,-0.05)
    # n = min(n, 0.05)
    # u[:,1]+=n

    # n = 0.01*randn(size(z,1),1)
    # n = max(n,-0.01)
    # n = min(n, 0.01)
    # u[:,2]+=n

    # y_vx=diff(reshape(z[:,1,:],size(z,1),size(z,3)),2)
    # y_vy=diff(reshape(z[:,2,:],size(z,1),size(z,3)),2)#/10
    # y_psi=diff(reshape(z[:,3,:],size(z,1),size(z,3)),2)

    y_vx = z[:,1,2] - z[:,1,1]
    y_vy = z[:,2,2] - z[:,2,1]
    y_psi = z[:,3,2] - z[:,3,1]

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
    # BACKSLASH OPERATOR WITHOUT REGULIZATION
    # c_Vx = A_vx\y_vx
    # c_Vy = A_vy\y_vy
    # c_Psi = A_psi\y_psi

    # BY IVS()
    # c_Vx = inv(A_vx'*A_vx)*A_vx'*y_vx
    # c_Vy = inv(A_vy'*A_vy)*A_vy'*y_vy
    # c_Psi = inv(A_psi'*A_psi)*A_psi'*y_psi

    # BACKSLASH WITH REGULARIZATION
    mu_Vx = zeros(3,3); mu_Vx[1,1] = 1e-5
    mu_Vy = zeros(4,4); mu_Vy[1,1] = 1e-5
    mu_Psi = zeros(3,3); mu_Psi[2,2] = 1e-5
    c_Vx = (A_vx'*A_vx+mu_Vx)\(A_vx'*y_vx)
    c_Vy = (A_vy'*A_vy+mu_Vy)\(A_vy'*y_vy)
    c_Psi = (A_psi'*A_psi+mu_Psi)\(A_psi'*y_psi)


    # println("c_Vx is $c_Vx")
    # println("c_Vx is $c_Vy")
    # println("c_Vx is $c_Psi")
    c_Vx[1] = max(-0.3,min(0.3,c_Vx[1]))
    # c_Vy[2] = max(-1,min(1,c_Vy[2]))
    # c_Vy[3] = max(-1,min(1,c_Vy[3]))

    return c_Vx, c_Vy, c_Psi
end

function find_SS(safeSetData::SafeSetData,selectedStates::SelectedStates,
                 s_curr::Float64,z_prev::Array{Float64,2},lapStatus::LapStatus,
                 modelParams::ModelParams,mpcParams::MpcParams,track::Track)

    s=s_curr; v=z_prev[2,4];
    N=mpcParams.N; dt=modelParams.dt
    Nl=selectedStates.Nl; Np_here=copy(selectedStates.Np/2)
    # Clear the safe set data of previous iteration
    selectedStates.selStates=Array{Float64}(0,6);
    selectedStates.statesCost=Array{Float64}(0,1);
    # target_s=s+v*dt*(N)   # 3 is a temporary shift number
    target_s=z_prev[end,1]+z_prev[end,4]*dt
    # for i=1:lapStatus.currentLap-1
    cost_correction = findmin(safeSetData.oldCost[lapStatus.currentLap-Nl:lapStatus.currentLap-1])[1]
    for i=1:Nl
        SS_curr=safeSetData.oldSS[:,:,lapStatus.currentLap-i]
        SS_next=safeSetData.oldSS[:,:,lapStatus.currentLap-i+1]
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
        # println(idx_s_end
        # println("Size of SS_curr is")
        # println(size(SS_curr))
        # println("Size of SS_cost is")
        # println(size(SScost_curr))
        if idx_s_start<1
            # println("lapStatus:",lapStatus)
            # println("oldCost:",safeSetData.oldCost[1:5])
            # println("<1:","idx_start:",idx_s_start," idx_end:",idx_s_end)
            # println(SS_next[1:idx_s_end,1])
            flag_array = isnan(SS_next[1:idx_s_end,:])
            flag = false
            for i = 1:length(flag_array)
                flag = flag | flag_array[i]
            end
            if flag
                # println("hahaha")
                SS_curr[cost+idx_s_start:cost,1]-=track.s
                selectedStates.selStates=vcat(selectedStates.selStates,SS_curr[cost+idx_s_start:cost,:],SS_curr[1:idx_s_end,:])
            else
                SS_curr[1:idx_s_end,1]+=track.s # correction when switching lap
                add_s = hcat(track.s*ones(idx_s_end),zeros(idx_s_end,size(SS_curr,2)-1))
                selectedStates.selStates=vcat(selectedStates.selStates,SS_curr[cost+idx_s_start:cost,:],reshape(SS_next[1:idx_s_end,:],idx_s_end,size(SS_curr,2))+add_s)
            end
            SScost_curr[1:idx_s_end]-=cost # correction when switching lap
            # SScost_curr[1:idx_s_end]-=cost_correction # correction when switching lap
            # selectedStates.statesCost=vcat(selectedStates.statesCost,SScost_curr[cost+idx_s_start:cost],SScost_curr[1:idx_s_end])
            selectedStates.statesCost=vcat(selectedStates.statesCost,(selectedStates.Np:-1:1)+cost)
        elseif idx_s_end>cost
            # println(">cost:","idx_start:",idx_s_start," idx_end:",idx_s_end,"lap cost",cost)
            SS_curr[1:idx_s_end-cost,1]+=track.s
            add_s = hcat(track.s*ones(idx_s_end-cost),zeros(idx_s_end-cost,size(SS_curr,2)-1))

            # selectedStates.selStates=vcat(selectedStates.selStates,SS_curr[idx_s_start:cost,:],SS_curr[1:idx_s_end-cost,:])
            selectedStates.selStates=vcat(selectedStates.selStates,SS_curr[idx_s_start:cost,:],reshape(SS_next[1:idx_s_end-cost,:],idx_s_end-cost,size(SS_curr,2))+add_s)
            SScost_curr[1:idx_s_end-cost]-=cost
            # SScost_curr[1:idx_s_end-cost]-=cost_correction
            # selectedStates.statesCost=vcat(selectedStates.statesCost,SScost_curr[idx_s_start:cost],SScost_curr[1:idx_s_end-cost])
            selectedStates.statesCost=vcat(selectedStates.statesCost,(selectedStates.Np:-1:1)+cost)
        else
            # println("else:","idx_start:",idx_s_start," idx_end:",idx_s_end,"lap cost",cost)
            if s>track.s-v*dt*(N)
                SS_curr[idx_s_start:idx_s_end,1]+=track.s
                add_s = hcat(track.s*ones(idx_s_end-idx_s_start+1),zeros(idx_s_end-idx_s_start+1,size(SS_curr,2)-1))
                # selectedStates.selStates=vcat(selectedStates.selStates,SS_curr[idx_s_start:idx_s_end,:])
                selectedStates.selStates=vcat(selectedStates.selStates,reshape(SS_next[idx_s_start:idx_s_end,:],idx_s_end-idx_s_start+1,size(SS_curr,2))+add_s)
                # selectedStates.statesCost=vcat(selectedStates.statesCost,SScost_curr[idx_s_start:idx_s_end])
                selectedStates.statesCost=vcat(selectedStates.statesCost,(selectedStates.Np:-1:1)+cost)
            else
                selectedStates.selStates=vcat(selectedStates.selStates,SS_curr[idx_s_start:idx_s_end,:])
                # selectedStates.statesCost=vcat(selectedStates.statesCost,SScost_curr[idx_s_start:idx_s_end])
                selectedStates.statesCost=vcat(selectedStates.statesCost,(selectedStates.Np:-1:1)+cost)
            end
        end
    end
    selectedStates.statesCost[:] -= cost_correction
    return selectedStates
end

function find_feature_space(solHistory::SolHistory,z_curr::Array{Float64,2},u_curr::Array{Float64,2},lapStatus::LapStatus,selectedStates::SelectedStates,posInfo::PosInfo)
    n_front = Int(round(0.66*selectedStates.feature_Np))
    n_rear  = Int(round(0.44*selectedStates.feature_Np))
    Nl = selectedStates.feature_Nl
    dummy_state=Array{Float64}(0,6)
    dummy_input=Array{Float64}(0,2)

    iden_z      =zeros(Nl*(n_front+n_rear+1)-1,3,2)
    iden_z_plot =zeros(Nl*(n_front+n_rear+1)-1,6)
    iden_u      =zeros(Nl*(n_front+n_rear+1)-1,2)

    for i = lapStatus.currentLap-Nl:lapStatus.currentLap-1
        state=reshape(solHistory.z[2:Int(solHistory.cost[i]),i,1,:],Int(solHistory.cost[i])-1,6)
        input=reshape(solHistory.u[1:Int(solHistory.cost[i])-1,i,2,:],Int(solHistory.cost[i])-1,2)
        state_rear = state[end-n_rear:end,:]
        state_rear[:,1] -= posInfo.s_target
        state_front = state[1:n_front,:]
        state_front[:,1] += posInfo.s_target
        state=vcat(state_rear,state,state_front)
        (~,idx)=findmin(abs(state[:,1]-z_curr[1]))
        # println(state[1:20,1])
        # println(z_curr)
        dummy_state=vcat(dummy_state,state[idx-n_rear:idx+n_front,:])
        dummy_input=vcat(dummy_input,input[idx-n_rear:idx+n_front,:])
    end
    iden_z[:,:,1] = dummy_state[1:end-1,4:6]
    iden_z[:,:,1] = dummy_state[2:end,4:6]
    iden_u[:,:] = dummy_input[1:end-1,:]
    iden_z_plot[:,:] = dummy_state[1:end-1,:]
    return iden_z, iden_u, iden_z_plot
end

function InitializeParameters(mpcParams::MpcParams,mpcParams_4s::MpcParams,mpcParams_pF::MpcParams,modelParams::ModelParams,mpcSol::MpcSol,
                              selectedStates::SelectedStates,oldSS::SafeSetData,oldTraj::OldTrajectory,mpcCoeff::MpcCoeff,mpcCoeff_dummy::MpcCoeff,
                              LMPC_LAP::Int64,delay_df::Int64,delay_a::Int64,N::Int64,BUFFERSIZE::Int64)
    simulator_flag   = true

    if simulator_flag == true   # if the BARC is in use

        mpcParams.N                 = N
        mpcParams.Q                 = [5.0,0.0,0.0,0.1,50.0,0.0]   # Q (only for path following mode)
        mpcParams.vPathFollowing    = 1.0                           # reference speed for first lap of path following
        mpcParams.R                 = 0*[10.0,10.0]                 # put weights on a and d_f
        mpcParams.QderivZ           = 1.0*[0,0.1,0.1,2,0.1,0.0]             # cost matrix for derivative cost of states
        mpcParams.QderivU           = 1.0*[1.0,1.0] #NOTE Set this to [5.0, 0/40.0]              # cost matrix for derivative cost of inputs
        mpcParams.Q_term_cost       = 0.05                        # scaling of Q-function
        mpcParams.delay_df          = delay_df                             # steering delay
        mpcParams.delay_a           = delay_a                             # acceleration delay
        mpcParams.Q_lane            = 10                      # weight on the soft constraint for the lane
        mpcParams.Q_slack           = 50.0*[1.0,1.0,1.0,1.0,1.0,1.0]#[20.0,10.0,10.0,30.0,80.0,50.0]  #s,ey,epsi,vx,vy,psiDot

        mpcParams_4s.N              = N
        mpcParams_4s.R              = 0.0*[1,1]
        mpcParams_4s.QderivZ        = 1.0*[0,0.1,0.1,2]
        mpcParams_4s.QderivU        = 1.0*[1,1]
        mpcParams_4s.Q_term_cost    = 3e-1 # scaling of Q-function
        mpcParams_4s.Q_lane         = 10.0 # weight on the soft constraint for the lane bounds
        mpcParams_4s.Q_slack        = 5.0*[1,1,1,1]
        mpcParams_4s.delay_df          = delay_df                             # steering delay
        mpcParams_4s.delay_a           = delay_a                             # acceleration delay

        mpcParams_pF.N              = N
        mpcParams_pF.Q              = [0.0,50.0,5.0,20.0]
        mpcParams_pF.R              = 0*[1.0,1.0]               # put weights on a and d_f
        mpcParams_pF.QderivZ        = 1.0*[0.0,0,1.0,0]           # cost matrix for derivative cost of states
        mpcParams_pF.QderivU        = 8*[2,1]                # cost matrix for derivative cost of inputs
        mpcParams_pF.vPathFollowing = 1                       # reference speed for first lap of path following
        mpcParams_pF.delay_df       = delay_df                         # steering delay (number of steps)
        mpcParams_pF.delay_a        = delay_a                         # acceleration delay

    elseif simulator_flag == false  # if the simulator is in use

        mpcParams.N                 = N
        mpcParams.Q                 = [5.0,0.0,0.0,0.1,50.0,0.0]   # Q (only for path following mode)
        mpcParams.vPathFollowing    = 1.0                           # reference speed for first lap of path following
        mpcParams.R                 = 0*[10.0,10.0]                 # put weights on a and d_f
        mpcParams.QderivZ           = 10.0*[0,0,1,1,1,1]             # cost matrix for derivative cost of states
        mpcParams.QderivU           = 100*[4.0,1.0] #NOTE Set this to [5.0, 0/40.0]              # cost matrix for derivative cost of inputs
        mpcParams.Q_term_cost       = 3                        # scaling of Q-function
        mpcParams.delay_df          = delay_df                             # steering delay
        mpcParams.delay_a           = delay_a                             # acceleration delay
        mpcParams.Q_lane            = 16                      # weight on the soft constraint for the lane
        mpcParams.Q_slack           = 1.0*[50.0,80.0,30.0,20.0,1.0,10.0]#[20.0,10.0,10.0,30.0,80.0,50.0]  #s,ey,epsi,vx,vy,psiDot

        mpcParams_4s.N              = N
        mpcParams_4s.R              = 0.0*[1,1]
        mpcParams_4s.QderivZ        = 1.0*[0,0.1,0.1,2]
        mpcParams_4s.QderivU        = 1.0*[1,1]
        mpcParams_4s.Q_term_cost    = 3e-1 # scaling of Q-function
        mpcParams_4s.Q_lane         = 10.0 # weight on the soft constraint for the lane bounds
        mpcParams_4s.Q_slack        = 5.0*[1,1,1,1]
        mpcParams_4s.delay_df          = delay_df                             # steering delay
        mpcParams_4s.delay_a           = delay_a                             # acceleration delay

        mpcParams_pF.N              = N
        mpcParams_pF.Q              = [0.0,20.0,10.0,10.0]
        mpcParams_pF.R              = 2*[1.0,1.0]               # put weights on a and d_f
        mpcParams_pF.QderivZ        = 1.0*[0.0,1.0,1.0,1.0]           # cost matrix for derivative cost of states
        mpcParams_pF.QderivU        = 0.1*[1,0.1]                # cost matrix for derivative cost of inputs
        mpcParams_pF.vPathFollowing = 1.5                       # reference speed for first lap of path following
        mpcParams_pF.delay_df       = delay_df                         # steering delay (number of steps)
        mpcParams_pF.delay_a        = delay_a                         # acceleration delay
    end

    selectedStates.Np           = 20        # please select an even number
    selectedStates.Nl           = 3         # Number of previous laps to include in the convex hull
    selectedStates.feature_Np   = 30        # Number of points from previous laps to do SYS_ID
    selectedStates.feature_Nl   = 2         # Number of previous laps to do SYS_ID 
    selectedStates.selStates    = zeros(selectedStates.Nl*selectedStates.Np,6)
    selectedStates.statesCost   = zeros(selectedStates.Nl*selectedStates.Np)

    modelParams.l_A             = 0.125
    modelParams.l_B             = 0.125
    modelParams.dt              = 0.1       
    modelParams.m               = 1.98
    modelParams.I_z             = 0.03
    modelParams.c_f             = 0.05       

    num_lap = 1 + selectedStates.Nl + LMPC_LAP

    oldSS.oldSS                 = NaN*ones(BUFFERSIZE,6,num_lap)    # contains data from previous laps usefull to build the safe set
    oldSS.oldSS_xy              = NaN*ones(BUFFERSIZE,4,num_lap)          
    oldSS.cost2target           = zeros(BUFFERSIZE,num_lap)         # cost to arrive at the target, i.e. how many iterations from the start to the end of the lap
    oldSS.oldCost               = ones(Int64,num_lap)               # contains costs of laps
    oldSS.count                 = ones(num_lap)*2                   # contains the counter for each lap

    oldTraj.oldTraj             = NaN*ones(5*BUFFERSIZE,7,num_lap)
    oldTraj.count               = ones(num_lap)

    mpcCoeff.c_Vx               = zeros(mpcParams.N,3)
    mpcCoeff.c_Vy               = zeros(mpcParams.N,4)
    mpcCoeff.c_Psi              = zeros(mpcParams.N,3)

    mpcCoeff_dummy.c_Vx         = zeros(1,3)
    mpcCoeff_dummy.c_Vy         = zeros(1,4)
    mpcCoeff_dummy.c_Psi        = zeros(1,3)

    mpcSol.z    = zeros(N+1,4)
    mpcSol.u    = hcat(0.5*ones(N),zeros(N))
    mpcSol.z[1,4] = 1.0 # give the vehcile some initial speed to simulate forward
    for i in 2:N+1
        mpcSol.z[i,4]=1.0
        mpcSol.z[i,1]=mpcSol.z[i-1,1]+0.1
    end
    mpcSol.a_x  = 0
    mpcSol.d_f  = 0
    mpcSol.df_his = zeros(delay_df) # DELAT COMES FROM TWO PARTS, ONLY THE SYSTEM DELAY NEEDS TO BE CONSIDERED
    mpcSol.a_his = zeros(delay_a) # DELAT COMES FROM TWO PARTS, ONLY THE SYSTEM DELAY NEEDS TO BE CONSIDERED
end

function s6_to_s4(z::Array{Float64})
    if size(z,1)==1
        z=vcat(z[1:3],sqrt(z[4]^2+z[5]^2))
    else # size(z,1)==2
        z=hcat(z[:,1:3],sqrt(z[:,4].^2+z[:,5].^2))
    # else
    #     error("Please check the input dimension of function \"6s_to_4s\" ")
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

function car_pre_dyn_true(z_curr::Array{Float64},u_sol::Array{Float64},track::Track,modelParams::ModelParams,outputDims::Int64)
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
            (z_dummy[i,:], Fy_dummy[i,:], a_slip_dummy[i,:])=car_sim_dyn_exact_true(z_dummy[i-1,:],u_sol[i-1,:],track,modelParams)
        end
        if outputDims==4
            z_dummy=s6_to_s4(z_dummy)
        end
    else
        error("Please check the u_sol dimension of function \"car_pre_dyn\" ")
    end

    return z_dummy, Fy_dummy, a_slip_dummy
end

function car_sim_dyn_exact_true(z::Array{Float64,2},u::Array{Float64,2},track::Track,modelParams::ModelParams)
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

function car_sim_dyn_true(z::Array{Float64},u::Array{Float64},dt::Float64,track::Track,modelParams::ModelParams)
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

    FyF = -pacejka_true(a_F)
    FyR = -pacejka_true(a_R)

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

function pacejka_true(a)
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

function pacejka(a)
    # B = 1.0             # This value determines the steepness of the curve
    # C = 1.25
    B = 6.0             # This value determines the steepness of the curve
    C = 1.6
    mu = 0.8            # Friction coefficient (responsible for maximum lateral tire force)
    m = 1.98
    g = 9.81
    D = mu * m * g/2
    # C_alpha_f = D*sin.(C*atan.(B*a))
    C_alpha_f = D*sin(C*atan(B*a))
    return C_alpha_f
end

function car_sim_iden_tv(z::Array{Float64},u::Array{Float64},dt::Float64,mpcCoeff::MpcCoeff,modelParams::ModelParams,track::Track)
    L_f = modelParams.l_A
    L_r = modelParams.l_B
    m   = modelParams.m
    I_z = modelParams.I_z
    c_f = modelParams.c_f

    c_Vx=mpcCoeff.c_Vx
    c_Vy=mpcCoeff.c_Vy
    c_Psi=mpcCoeff.c_Psi

    idx=Int(ceil(z[1]/track.ds))+1 # correct the starting original point idx problem
    idx>track.n_node ? idx=idx%track.n_node : nothing
    idx<=0 ? idx+=track.n_node : nothing

    c=track.curvature[idx]
    dsdt = (z[4]*cos(z[3]) - z[5]*sin(z[3]))/(1-z[2]*c)

    z_next=copy(z)

    z_next[1]  = z_next[1] + dt * dsdt                                # s
    z_next[2]  = z_next[2] + dt * (z[4]*sin(z[3]) + z[5]*cos(z[3]))   # eY
    z_next[3]  = z_next[3] + dt * (z[6]-dsdt*c)                       # ePsi
    z_next[4]  = z_next[4] + c_Vx[1]*z[5]*z[6] + c_Vx[2]*z[4] + c_Vx[3]*u[1]                           # vx
    z_next[5]  = z_next[5] + c_Vy[1]*z[5]/z[4] + c_Vy[2]*z[4]*z[6] + c_Vy[3]*z[6]/z[4] + c_Vy[4]*u[2]  # vy
    z_next[6]  = z_next[6] + c_Psi[1]*z[6]/z[4] + c_Psi[2]*z[5]/z[4] + c_Psi[3]*u[2]                   # psiDot
    return z_next
end

# FUNCTIONS FOR GP REGRESSION
function covar_fun(z1::Array{Float64,2},z2::Array{Float64,2})
    # input: [vx,vy,psi_dot,a,df]
    z = z1-z2
    # L[1,1]=0.1;
    # L[2,2]=5;
    # L[3,3]=5;
    # L[4,4]=0.1;
    # L[5,5]=5
    k = exp(-0.5*(0.1*z[1]^2+5*z[2]^2+5*z[3]^2+0.1*z[4]^2+5*z[5]^2))
    return k
end

function regre(z::Array{Float64,2},u::Array{Float64,2},e::Array{Float64,1},s::Array{Float64,2},i::Array{Float64,2})
    # find the closest local feature points
    dummy_norm = zeros(size(s,1),2)
    state = hcat(z,u)
    feature_state = hcat(s,i)
    dist = state[4:8]'.-feature_state[:,4:8]
    # We have 5 dimensions for distance calculation
    dummy_norm[:,1] = dist[:,1].^2+(5*dist[:,5]).^2#+dist[:,3].^2+dist[:,4].^2+dist[:,5].^2
    dummy_norm[:,2] = 1:size(dummy_norm,1)
    dummy_norm=sortrows(dummy_norm) # pick up the first minimum Np points

    # do local GP
    # pick the first 20 points for local GP
    num = 20
    K=zeros(num,num)
    k=zeros(num)
    y=zeros(num)
    for i = 1:num
        for j=1:num
            z = feature_state[Int(dummy_norm[i,2]),4:8] - feature_state[Int(dummy_norm[j,2]),4:8]
            K[i,j] = exp(-0.5*(0.1*z[1]^2+5*z[2]^2+5*z[3]^2+0.1*z[4]^2+5*z[5]^2))
            # K[i,j]=covar_fun(feature_state[Int(dummy_norm[i,2]),4:8],feature_state[Int(dummy_norm[j,2]),4:8])
        end
        z = feature_state[Int(dummy_norm[i,2]),4:8]-state[4:8]'
        k[i] = exp(-0.5*(0.1*z[1]^2+5*z[2]^2+5*z[3]^2+0.1*z[4]^2+5*z[5]^2))
        # k[i]=covar_fun(feature_state[Int(dummy_norm[i,2]),4:8],state[4:8]')
        y[i]=e[Int(dummy_norm[i,2])]
        # println(feature_state[Int(dummy_norm[i,2]),4:8])
    end

    local_e = k'*(K\y)
    return local_e[1]
end

function GP_prepare(e::Array{Float64,1},s::Array{Float64,2},i::Array{Float64,2})
    # find the closest local feature points
    feature_state = hcat(s,i) 
    num = size(feature_state,1)
    # pick the first 20 points for local GP
    # y=zeros(num)
    # L = eye(5);
    # L[1,1]=0.1;
    # L[2,2]=5;
    # L[3,3]=5;
    # L[4,4]=0.1;
    # L[5,5]=5
    Z=zeros(num,num)
    for i = 1:num
        for j=1:num
            z = feature_state[i,4:8]-feature_state[j,4:8]
            Z[i,j] = (0.5*z[1]^2+5*z[2]^2+5*z[3]^2+0.5*z[4]^2+5*z[5]^2)
        end
        # y[i]=e[i]
    end
    K=exp(-0.5*Z)
    return K\e
end

function GP_full_vy(z::Array{Float64,2},u::Array{Float64,2},feature_state::Array{Float64,2},GP_prepare::Array{Float64,1})
    state = hcat(z,u)
    z = feature_state[:,4:8].-state[1,4:8]
    Z = 0.5*z[:,1].^2+5*z[:,2].^2+5*z[:,3].^2+0.5*z[:,4].^2+5*z[:,5].^2
    # Z = z[:,1].^2+z[:,2].^2+z[:,3].^2+z[:,4].^2+z[:,5].^2
    k = 0.5^2*exp(-0.5*Z)
    GP_e = k'*GP_prepare
    return GP_e[1]
end

function GP_full_psidot(z::Array{Float64,2},u::Array{Float64,2},feature_state::Array{Float64,2},GP_prepare::Array{Float64,1})
    state = hcat(z,u)
    z = feature_state[:,4:8].-state[1,4:8]
    Z = 0.5*z[:,1].^2+5*z[:,2].^2+5*z[:,3].^2+0.5*z[:,4].^2+5*z[:,5].^2
    # Z = z[:,1].^2+z[:,2].^2+z[:,3].^2+z[:,4].^2+z[:,5].^2
    k = 0.5^2*exp(-0.5*Z)
    GP_e = k'*GP_prepare
    return GP_e[1]
end

function createTrack(name::ASCIIString)
    # RACING TRACK DATA
    if name == "race"
        # TRACK USED IN MY SIMULATION
        track_data=[80 0;
                    120 -pi/2;
                    80 0;
                    220 -pi*0.85;
                    105 pi/15;
                    300  pi*1.15;
                    240  -pi*0.865;
                    100 0;
                    120 -pi/2;
                    153 0;
                    120 -pi/2;
                    211 0]
    elseif name == "3110"
        # EXPERIEMENT TRACK DATA
        num = 100
        track_data=[Int(ceil(2*80)) 0;
                    Int(ceil(2*num)) -pi/2;
                    Int(ceil(2*(80+47))) 0;
                    Int(ceil(2*num)) -pi/2;
                    Int(ceil(2*50)) 0;
                    Int(ceil(2*num)) -pi/2;
                    Int(ceil(2*4)) 0;
                    Int(ceil(2*num)) pi/2;
                    Int(ceil(2*30)) 0;
                    Int(ceil(2*num)) -pi/2;
                    Int(ceil(2*4)) 0;
                    Int(ceil(2*num)) -pi/2;
                    Int(ceil(2*(71+48))) 0]
    elseif name == "basic"
        # Basic experiment track
        # track_data = [Int(ceil(3*60)) 0;
        #               Int(ceil(3*80)) -pi/2;
        #               Int(ceil(3*20)) 0;
        #               Int(ceil(3*80)) -pi/2;
        #               Int(ceil(3*40)) pi/10;
        #               Int(ceil(3*60)) -pi/5;
        #               Int(ceil(3*40)) pi/10;
        #               Int(ceil(3*80)) -pi/2;
        #               Int(ceil(3*20)) 0;
        #               Int(ceil(3*80)) -pi/2;
        #               Int(ceil(3*75)) 0]

        track_data = [Int(ceil(2.8*40)) 0;
                      Int(ceil(2.8*120)) -pi/2;
                      Int(ceil(2.8*5)) 0;
                      Int(ceil(2.8*120)) -pi/2;
                      Int(ceil(2.8*80)) 0;
                      Int(ceil(2.8*120)) -pi/2;
                      Int(ceil(2.8*5)) 0;
                      Int(ceil(2.8*120)) -pi/2;
                      Int(ceil(2.8*40)) 0]
    elseif name == "MSC_lab"    
        # TRACK TO USE IN THE SMALL EXPERIMENT ROOM
        track_data = [Int(ceil(1.5*3*10)) 0;
                      Int(ceil(1.5*3*120)) -pi;
                      Int(ceil(1.5*3*20)) 0;
                      Int(ceil(1.5*3*120)) -pi;
                      Int(ceil(1.5*3*10)) 0]
    elseif name == "feature"
        # FEATURE TRACK DATA
        ds = 0.01
        v = 2.5    
        max_a=7.6;
        R=v^2/max_a
        max_c=1/R
        angle=(pi+pi/2)-0.105
        R_kin = 0.8
        num_kin = Int(round(angle/ ( ds/R_kin ) * 2))
        num = max(Int(round(angle/ ( ds/R ) * 2)),num_kin)
        num=Int(ceil(num*1.3))
        track_data=[num -angle;
                    num  angle]
    else
        error("Please input the correct track name")
    end
    return track_data
end