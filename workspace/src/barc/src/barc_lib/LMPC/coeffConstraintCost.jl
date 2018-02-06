# This function evaluates and returns the coefficients for constraints and cost which are used later in the MPC formulation
# Inputs are:
# oldTraj   -> contains information about previous trajectories and Inputs
# mpcCoeff  -> contains information about previous coefficient results
# posInfo   -> contains information about the car's current position along the track
# mpcParams -> contains information about the MPC formulation (e.g. Q, R)
# stateIn   -> current state of the car
# inputIn   -> last input to the system

# structure of oldTrajectory: 1st dimension = state number, 2nd dimension = step number (time equiv.), 3rd dimennsion = lap number

# z[1] = xDot
# z[2] = yDot
# z[3] = psiDot
# z[4] = ePsi
# z[5] = eY
# z[6] = s

function coeffConstraintCost(oldTraj::OldTrajectory, mpcCoeff::MpcCoeff, posInfo::PosInfo, mpcParams::MpcParams,lapStatus::LapStatus,
                             selectedStates::SelectedStates,oldSS::SafeSetData,obs::Array{Float64},obstacle::Obstacle)
    # this computes the coefficients for the cost and constraints

    # Outputs: 
    # coeffConst
    # coeffCost

    # Read Inputs
    s               = posInfo.s
    s_target        = posInfo.s_target

    dt = 0.1
    N  = mpcParams.N

    Np              = selectedStates.Np
    Nl              = selectedStates.Nl

    # Parameters
    Order           = mpcCoeff.order                # interpolation order for cost and constraints
    pLength         = mpcCoeff.pLength              # interpolation length for polynomials
    delay_df        = mpcParams.delay_df
    #delay           = 2

    #coeffCost       = zeros(Order+1,2)              # polynomial coefficients for cost
    #coeffConst      = zeros(Order+1,2,5)            # nz-1 beacuse no coeff for s

    n_laps_sysID    = 2                             # number of previous laps that are used for sysID

    selected_laps = zeros(Int64,Nl)
    # selected_laps[1] = lapStatus.currentLap-1                                   # use previous lap
    # selected_laps[2] = lapStatus.currentLap-2                                   # and the one before
    # if lapStatus.currentLap >= 5
    #     selected_laps[2] = indmin(oldTraj.oldCost[2:lapStatus.currentLap-2])+1      # and the best from all previous laps
    # end

    for i = 1:Nl
        selected_laps[i] = lapStatus.currentLap-i    # use previous lap
    end

    if lapStatus.currentLap >= 5
        selected_laps[Nl] = indmin(oldSS.oldCost[1:lapStatus.currentLap-2])      # and the best from all previous laps
    end

    # Select the old data
    oldxDot         = oldTraj.oldTraj[:,1,selected_laps]::Array{Float64,3}
    oldyDot         = oldTraj.oldTraj[:,2,selected_laps]::Array{Float64,3}
    oldpsiDot       = oldTraj.oldTraj[:,3,selected_laps]::Array{Float64,3}
    oldePsi         = oldTraj.oldTraj[:,4,selected_laps]::Array{Float64,3}
    oldeY           = oldTraj.oldTraj[:,5,selected_laps]::Array{Float64,3}
    oldS            = oldTraj.oldTraj[:,6,selected_laps]::Array{Float64,3}
    oldacc          = oldTraj.oldTraj[:,7,selected_laps]::Array{Float64,3}
    olda            = oldTraj.oldInput[:,1,selected_laps]::Array{Float64,3}
    olddF           = oldTraj.oldInput[:,2,selected_laps]::Array{Float64,3}
    #olddF           = smooth(olddF,5)

    oldS_safeSet    = oldSS.oldSS[:,6,selected_laps]::Array{Float64,3}
    #println("safe set= ",oldS_safeSet[1:20,:,:])

    N_points        = size(oldTraj.oldTraj,1)     # second dimension = length (=buffersize)
    N_points2       = size(oldSS.oldSS,1)


    s_total::Float64        # initialize
    DistS::Array{Float64}   # initialize
    idx_s::Array{Int64}     # initialize
    idx_s_target        = 0
    dist_to_s_target    = 0
    qLength             = 0
    vec_range::Tuple{UnitRange{Int64},UnitRange{Int64}}
    bS_Vector::Array{Float64}
    s_forinterpy::Array{Float64}

    # Compute the total s (current position along track)
    s_total = s % s_target

    # Compute the index
    DistS = ( s_total - oldS ).^2
    DistS2 = ( s_total - oldS_safeSet ).^2


    idx_s = findmin(DistS,1)[2]              # contains both indices for the closest distances for both oldS !!
    idx_s2= findmin(DistS2,1)[2]

    idx_s2 = idx_s2 + selectedStates.shift

    # Propagate the obstacle for the prediction horizon

    obs_prop_s  = obs[1,1,:] + dt*N*obs[1,3,:]
    obs_prop_ey = obs[1,2,:]


    #######################################################################

    for j = 0:(Nl-1)

        selectedStates.selStates[i=(j*Np)+1:(j+1)*Np,m=1:6] = oldSS.oldSS[i=idx_s2[j+1]-(j*N_points2):idx_s2[j+1]+Np-(j*N_points2)-1,i=1:6,selected_laps[j+1]]  # select the states from lap j...
        
        selectedStates.statesCost[i=(j*Np)+1:(j+1)*Np] = oldSS.cost2target[i=idx_s2[j+1]-(j*N_points2):idx_s2[j+1]-(j*N_points2)+Np-1,selected_laps[j+1]]  # and their cost

        if obstacle.obstacle_active == true   # if the obstacles are on the track, check if any of the selected states interferes with the propagated obstacle
            for n=1:obstacle.n_obs
                ellipse_check = (((selectedStates.selStates[i=(j*Np)+1:(j+1)*Np,6]-obs_prop_s[n])/obstacle.r_s).^2) + (((selectedStates.selStates[i=(j*Np)+1:(j+1)*Np,5]-obs_prop_ey[n])/obstacle.r_ey).^2)
                
                if any(x->x<=1, ellipse_check) == true  # if any of the selected states is in the ellipse
                    #println("flag**************************************************************************************************************")
                    index = find(ellipse_check.<=1)     # find all the states in the ellipse 
                    
                    mpcParams.Q_obs[i=(j*Np)+(index[1]-obstacle.inv_step)+1:(j+1)*Np] =  10   # and set the values of the weight to 10, so that they are excluded from optimization
                    println("Q_obs= ",mpcParams.Q_obs)
                end
            end
        end     
        
    end

        #println("Q_obs= ",mpcParams.Q_obs)


    ##########################################################################
    if selectedStates.version == true
        vec_range = (idx_s[1]:idx_s[1]+pLength,idx_s[2]:idx_s[2]+pLength)

        # Create the vectors used for the interpolation
        # bS_vector contains the s-values for later interpolation
        bS_Vector       = zeros(pLength+1,2)
        for i=1:pLength+1
            bS_Vector[i,1] = oldS[vec_range[1][i]]
            bS_Vector[i,2] = oldS[vec_range[2][i]]
        end
        # if norm(bS_Vector[1,1]-s_total) > 0.3 || norm(bS_Vector[1,2]-s_total) > 0.3
        #     warn("Couldn't find a close point to current s.")
        # end

        # The states are parametrized with resprect to the curvilinear abscissa,
        # so we select the point used for the interpolation. Need to subtract an
        # offset to be coherent with the MPC formulation
        s_forinterpy   = bS_Vector
        if s_total < 0
            s_forinterpy += s_target
        end

        # Create the Matrices for the interpolation
        MatrixInterp = zeros(pLength+1,Order+1,2)

        for k = 0:Order
            MatrixInterp[:,Order+1-k,:] = s_forinterpy[:,:].^k
        end
        
        # Compute the coefficients
        #coeffConst = zeros(Order+1,2,5)
        for i=1:2
            mpcCoeff.coeffConst[:,i,1]    = MatrixInterp[:,:,i]\oldxDot[vec_range[i]]
            mpcCoeff.coeffConst[:,i,2]    = MatrixInterp[:,:,i]\oldyDot[vec_range[i]]
            mpcCoeff.coeffConst[:,i,3]    = MatrixInterp[:,:,i]\oldpsiDot[vec_range[i]]
            mpcCoeff.coeffConst[:,i,4]    = MatrixInterp[:,:,i]\oldePsi[vec_range[i]]
            mpcCoeff.coeffConst[:,i,5]    = MatrixInterp[:,:,i]\oldeY[vec_range[i]]
        end

        # Finished with calculating the constraint coefficients
        
        # Now compute the final cost coefficients

        # The Q-function contains for every point in the sampled safe set the minimum cost-to-go-value
        # These values are calculated for both old trajectories
        # The vector bQfunction_Vector contains the cost at each point in the interpolated area to reach the finish line
        # From this vector, polynomial coefficients coeffCost are calculated to approximate this cost
        for i=1:2   
                dist_to_s_target  = oldTraj.oldCost[selected_laps[i]] + oldTraj.idx_start[selected_laps[i]] - (idx_s[i]-N_points*(i-1))  # number of iterations from idx_s to s_target
                bQfunction_Vector = collect(linspace(dist_to_s_target,dist_to_s_target-pLength,pLength+1))    # build a vector that starts at the distance and
                                                                                                        # decreases in equal steps
                mpcCoeff.coeffCost[:,i]    = MatrixInterp[:,:,i]\bQfunction_Vector           # interpolate this vector with the given s
        end
    end

    # --------------- SYSTEM IDENTIFICATION --------------- #
    # ----------------------------------------------------- #

    cC = oldTraj.count[lapStatus.currentLap]-1      # current count
    cL = lapStatus.currentLap                       # current lap
    #vec_range_ID2   = cC-n_prev:cC-1                # index range for current lap
    
    # TODO:*******************
    # CHECK IF DIFFERENCES ARE ALIGNED PROPERLY
    # ADD 2-step-delay to sysID
    # ************************
    # collect indices for system ID
    n_sys_ID_prev = 60              # steps of sysID before current point in previous laps
    n_sys_ID_post = 60              # steps of sysID after current point in previous laps
    n_sys_ID_prev_c = 30            # steps of sysID before current point in current lap

    freq_ratio = 5                  # ratio between the frequency at which the informations about the position are updated in the topic and the frequency at which the controller calculates and updates the control input
    # vec_range_ID    = ()
    # for i=1:n_laps_sysID
    #     vec_range_ID    = tuple(vec_range_ID...,idx_s[i]-n_prev:idx_s[i]+n_ahead)     # related index range
    # end

    sysID_idx_diff = idx_s[1]-n_sys_ID_prev*freq_ratio-1 + (1:freq_ratio:(n_sys_ID_prev+n_sys_ID_post+1)*freq_ratio)         # these are the indices that are used for differences
    sysID_idx = sysID_idx_diff[1:end-1]

    sysID_idx_diff2 = idx_s[2]-n_sys_ID_prev*freq_ratio-1 + (1:freq_ratio:(n_sys_ID_prev+n_sys_ID_post+1)*freq_ratio)         # these are the indices that are used for differences
    sysID_idx2 = sysID_idx_diff2[1:end-1]

    sysID_idx_diff_c = cC - (n_sys_ID_prev_c+1)*freq_ratio-1 + (1:freq_ratio:(n_sys_ID_prev_c+1)*freq_ratio)
    sysID_idx_c = sysID_idx_diff_c[1:end-1]

    sz1 = size(sysID_idx,1)
    sz2 = size(sysID_idx_c,1)
    sz2 = 0
    sz = sz1 + sz2

    # psiDot
    y_psi = zeros((2*sz1+sz2)*freq_ratio)
    A_psi = zeros((2*sz1+sz2)*freq_ratio,3)
    for i=0:freq_ratio-1
        y_psi[(1:sz1)+i*sz1]            = diff(oldpsiDot[sysID_idx_diff+i])
        A_psi[(1:sz1)+i*sz1,:]          = [oldpsiDot[sysID_idx+i]./oldxDot[sysID_idx+i] oldyDot[sysID_idx+i]./oldxDot[sysID_idx+i] olddF[sysID_idx+i]]
        #y_psi[(1:sz2)+i*sz2+freq_ratio*sz1]      = diff(oldTraj.oldTraj[sysID_idx_diff_c+i,3,cL])
        #A_psi[(1:sz2)+i*sz2+freq_ratio*sz1,:]    = [oldTraj.oldTraj[sysID_idx_c+i,3,cL]./oldTraj.oldTraj[sysID_idx_c+i,1,cL] oldTraj.oldTraj[sysID_idx_c+i,2,cL]./oldTraj.oldTraj[sysID_idx_c+i,1,cL] oldTraj.oldInput[sysID_idx_c+i-delay_df*freq_ratio,2,cL]]
        y_psi[(1:sz1)+i*sz1+freq_ratio*sz1+freq_ratio*sz2]   = diff(oldpsiDot[sysID_idx_diff2+i])
        A_psi[(1:sz1)+i*sz1+freq_ratio*sz1+freq_ratio*sz2,:] = [oldpsiDot[sysID_idx2+i]./oldxDot[sysID_idx2+i] oldyDot[sysID_idx2+i]./oldxDot[sysID_idx2+i] olddF[sysID_idx2+i]]
    end

    # xDot
    y_xDot = zeros((2*sz1+sz2)*freq_ratio)
    A_xDot = zeros((2*sz1+sz2)*freq_ratio,3)
    for i=0:freq_ratio-1
        y_xDot[(1:sz1)+i*sz1]            = diff(oldxDot[sysID_idx_diff+i])
        A_xDot[(1:sz1)+i*sz1,:]          = [oldyDot[sysID_idx+i].*oldpsiDot[sysID_idx+i] oldxDot[sysID_idx+i] oldacc[sysID_idx+i]]
        #y_xDot[(1:sz2)+i*sz2+freq_ratio*sz1]      = diff(oldTraj.oldTraj[sysID_idx_diff_c+i,1,cL])
        #A_xDot[(1:sz2)+i*sz2+freq_ratio*sz1,:]    = [oldTraj.oldTraj[sysID_idx_c+i,2,cL].*oldTraj.oldTraj[sysID_idx_c+i,3,cL] oldTraj.oldTraj[sysID_idx_c+i,1,cL]  oldTraj.oldTraj[sysID_idx_c+i,7,cL]]
        y_xDot[(1:sz1)+i*sz1+freq_ratio*(sz1+sz2)]   = diff(oldxDot[sysID_idx_diff2+i])
        A_xDot[(1:sz1)+i*sz1+freq_ratio*(sz1+sz2),:] = [oldyDot[sysID_idx2+i].*oldpsiDot[sysID_idx2+i] oldxDot[sysID_idx2+i] oldacc[sysID_idx2+i]]
    end

    # yDot
    y_yDot = zeros((2*sz1+sz2)*freq_ratio)
    A_yDot = zeros((2*sz1+sz2)*freq_ratio,4)
    for i=0:freq_ratio-1
        y_yDot[(1:sz1)+i*sz1]            = diff(oldyDot[sysID_idx_diff+i])
        A_yDot[(1:sz1)+i*sz1,:]          = [oldyDot[sysID_idx+i]./oldxDot[sysID_idx+i] oldpsiDot[sysID_idx+i].*oldxDot[sysID_idx+i] oldpsiDot[sysID_idx+i]./oldxDot[sysID_idx+i] olddF[sysID_idx+i-delay_df*freq_ratio]]
        #y_yDot[(1:sz2)+i*sz2+freq_ratio*sz1]      = diff(oldTraj.oldTraj[sysID_idx_diff_c+i,2,cL])
        #A_yDot[(1:sz2)+i*sz2+freq_ratio*sz1,:]    = [oldTraj.oldTraj[sysID_idx_c+i,2,cL]./oldTraj.oldTraj[sysID_idx_c+i,1,cL] oldTraj.oldTraj[sysID_idx_c+i,3,cL].*oldTraj.oldTraj[sysID_idx_c+i,1,cL] oldTraj.oldTraj[sysID_idx_c+i,3,cL]./oldTraj.oldTraj[sysID_idx_c+i,1,cL] oldTraj.oldInput[sysID_idx_c+i-delay_df*freq_ratio,2]]
        y_yDot[(1:sz1)+i*sz1+freq_ratio*(sz1+sz2)]   = diff(oldyDot[sysID_idx_diff2+i])
        A_yDot[(1:sz1)+i*sz1+freq_ratio*(sz1+sz2),:] = [oldyDot[sysID_idx2+i]./oldxDot[sysID_idx2+i] oldpsiDot[sysID_idx2+i].*oldxDot[sysID_idx2+i] oldpsiDot[sysID_idx2+i]./oldxDot[sysID_idx2+i] olddF[sysID_idx2+i-delay_df*5]]
    end

    # if any(isnan,y_yDot)            # check if any value in the y_yDot value is NaN
    #     println(y_yDot)
    #     warn("NaN value detected in y_yDot! Press to continue...")
    # end
    # if any(isnan,coeffCost)
    #     println(coeffCost)
    #     warn("NaN value detected in coeffCost! Press to continue...")
    # end
    # if any(isnan,coeffConst)
    #     println(coeffCost)
    #     warn("NaN value detected in coeffConst! Press to continue...")
    # end

    # mpcCoeff.c_Psi = zeros(3)
    # mpcCoeff.c_Vx = zeros(4)
    # mpcCoeff.c_Vy = zeros(4)

    mpcCoeff.c_Psi = (A_psi'*A_psi)\A_psi'*y_psi
    mpcCoeff.c_Vx  = (A_xDot'*A_xDot)\A_xDot'*y_xDot         # the identity matrix is used to scale the coefficients
    mpcCoeff.c_Vy  = (A_yDot'*A_yDot)\A_yDot'*y_yDot
    
    # if det(A_psi'*A_psi) != 0
    # else
    #     println("det y_psi = 0")
    # end
    # if det(A_xDot'*A_xDot) != 0
    # else
    #     println("det vx = 0")
    # end
    # if det(A_yDot'*A_yDot) != 0
    # else
    #     println("det vy = 0")
    # end
    #println("Done. c_Psi = $(mpcCoeff.c_Psi)")
    #mpcCoeff.c_Psi = [-1.0,-4.5,5.8]
    #mpcCoeff.c_Vy  = [-1.2,-0.04,-0.01,0.6]
    #mpcCoeff.c_Psi = [-0.26682109207165566,-0.013445078992161885,1.2389672517023724]
    #mpcCoeff.c_Vy = [-0.006633028965076818,-0.02997779668710061,0.005781203137095575,0.10642934131787765]
    #mpcCoeff.coeffCost  = coeffCost
    #mpcCoeff.coeffConst = coeffConst
    nothing
end

# Notes about oldTrajectory:
# oldTrajectory[1:prebuf] = states before finish line (s < 0)
# oldTrajectory[prebuf+1:prebuf+cost] = states between start and finish line (0 <= s < s_target)
# oldTrajectory[prebuf+cost+1:prebuf+cost+postbuf] = states after finish line (s_target <= s)
# once one lap is over, the states are saved (interval prebuf:s_target)
# during the next lap, the states behind the finish line are appended (interval s_target:postbuf)
