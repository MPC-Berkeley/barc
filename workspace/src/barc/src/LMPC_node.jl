#!/usr/bin/env julia

using RobotOS
@rosimport barc.msg: ECU, pos_info, mpc_solution
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using geometry_msgs.msg
using JuMP
using Ipopt
using JLD

# log msg
include("barc_lib/classes.jl")
include("barc_lib/LMPC/MPC_models.jl")
include("barc_lib/LMPC/functions.jl")
include("barc_lib/LMPC/solveMpcProblem.jl")
include("barc_lib/simModel.jl")

function SE_callback(msg::pos_info,acc_f::Array{Float64},lapStatus::LapStatus,posInfo::PosInfo,mpcSol::MpcSol,oldTraj::OldTrajectory,z_est::Array{Float64,1},x_est::Array{Float64,1})
    z_est[:]                  = [msg.v_x,msg.v_y,msg.psiDot,msg.epsi,msg.ey,msg.s,acc_f[1]] # the last variable is filtered acceleration
    # z_est[:]                  = [msg.v_x,msg.v_y,msg.psiDot,msg.epsi,msg.ey,msg.s,acc_f[1]] # the last variable is filtered acceleration
    x_est[:]                  = [msg.x,msg.y,msg.psi,msg.v]
    
    if z_est[6] <= lapStatus.s_lapTrigger && lapStatus.switchLap
        lapStatus.nextLap = true
        lapStatus.switchLap = false
    elseif z_est[6] > lapStatus.s_lapTrigger
        lapStatus.switchLap = true
    end

    # save current state in oldTraj
    oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap],:,lapStatus.currentLap] = z_est
    oldTraj.count[lapStatus.currentLap] += 1
end

function ST_callback(msg::pos_info,z_true::Array{Float64,1})
    z_true[:] = [msg.x,msg.y,msg.v_x,msg.v_y,msg.psi,msg.psiDot]
    # println("z_true from LMPC node",round(z_true,4))
end

# This is the main function, it is called when the node is started.
function main()
    println("Starting LMPC node.")
    const BUFFERSIZE       = 500
    const LMPC_LAP         = 30

    const PF_FLAG          = true  # true:only pF,     false:1 warm-up lap and LMPC

    const LMPC_FLAG        = false   # true:IDEN_MODEL,  false:IDEN_KIN_LIN_MODEL(if all flags are false)
    const LMPC_DYN_FLAG    = false   # true:DYN_LIN_MODEL, false:IDEN_KIN_LIN_MODEL(if all flags are false)
    const LMPC_KIN_FLAG    = true   # true:KIN_MODEL,  false:IDEN_KIN_LIN_MODEL(if all flags are false)

    const FEATURE_FLAG     = false  # true:8-shape,     false:history (this requires the corresponding change in 3 palces)
    const NORM_SPACE_FLAG  = true  # true:2-norm,      false: space critirion

    const TI_TV_FLAG       = true   # true:TI,          false:TV

    const GP_LOCAL_FLAG    = false  # true:local GPR
    const GP_FULL_FLAG     = false  # true:full GPR
    
    const GP_HISTORY_FLAG  = false  # true: GPR data is from last laps, false: GP data is from data base.

    const N                = 10
    const delay_df         = 3
    const delay_a          = 1
    
    if PF_FLAG
        file_name = "PF"
    else
        if LMPC_FLAG # CONTROLLER FLAG
            if TI_TV_FLAG
                file_name = "SYS_ID_TI"
            else
                file_name = "SYS_ID_TV"
            end
        elseif LMPC_DYN_FLAG
            file_name = "DYN_LIN"
        elseif LMPC_KIN_FLAG
        	file_name = "KIN"
        else
            file_name = "SYS_ID_KIN_LIN"
            if TI_TV_FLAG
                file_name *= "_TI"
            else
                file_name *= "_TV"
            end
        end
    end
    if GP_LOCAL_FLAG
        file_name *= "_LOCAL_GP"
    elseif GP_FULL_FLAG
        file_name *= "_FULL_GP"
    end

    if get_param("sim_flag")
        folder_name = "simulations"
    else
        folder_name = "experiments"
    end



    PF_FLAG ? println("Path following") : println("Learning MPC")
    FEATURE_FLAG ? println("Feature data: 8-shape database") : println("Feature data: History data base")
    TI_TV_FLAG ? println("Time invariant SYS_ID") : println("Time variant SYS_ID")
    println("N=$N, delay_df=$delay_df, delay_a=$delay_a")
    
    track_data       = createTrack("3110")
    track            = Track(track_data)
    track_fe         = createTrack("feature")
    track_f          = Track(track_fe)
    oldTraj          = OldTrajectory()
    posInfo          = PosInfo();  posInfo.s_target=track.s;
    lapStatus        = LapStatus(1,1,false,false,0.3)
    mpcCoeff         = MpcCoeff()
    mpcCoeff_dummy   = MpcCoeff()
    mpcSol           = MpcSol()
    modelParams      = ModelParams()
    mpcParams_pF     = MpcParams()
    mpcParams        = MpcParams()
    mpcParams_4s     = MpcParams()
    selectedStates   = SelectedStates()
    oldSS            = SafeSetData()
    # oldSS_true       = SafeSetData()
    # oldSS_true.oldSS = NaN*ones(BUFFERSIZE,6,10)    # contains data from previous laps usefull to build the safe set
    z_est            = zeros(7)          # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
    z_true           = zeros(6)          # (xDot, yDot, psiDot)
    x_est            = zeros(4)          # (x, y, psi, v)
    cmd              = ECU()             # CONTROL SIGNAL MESSAGE INITIALIZATION
    mpcSol_to_pub    = mpc_solution()    # MPC SOLUTION PUBLISHING MESSAGE INITIALIZATION
    InitializeParameters(mpcParams,mpcParams_4s,mpcParams_pF,modelParams,mpcSol,
                         selectedStates,oldSS,oldTraj,mpcCoeff,mpcCoeff_dummy,
                         LMPC_LAP,delay_df,delay_a,N,BUFFERSIZE)
    z_prev           = mpcSol.z
    u_prev           = mpcSol.u
    GP_e_vy          = zeros(mpcParams.N)
    GP_e_psidot      = zeros(mpcParams.N)
    d_f_lp = 0.0

    # NODE INITIALIZATION
    init_node("mpc_traj")
    loop_rate   = Rate(1/modelParams.dt)
    pub         = Publisher("ecu", ECU, queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
    mpcSol_pub  = Publisher("mpc_solution", mpc_solution, queue_size=1)::RobotOS.Publisher{barc.msg.mpc_solution}; acc_f = [0.0]
    s1          = Subscriber("pos_info", pos_info, SE_callback, (acc_f,lapStatus,posInfo,mpcSol,oldTraj,z_est,x_est),queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
    s2          = Subscriber("real_val", pos_info, ST_callback, (z_true,),queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}

    num_lap             = LMPC_LAP+1+max(selectedStates.Nl,selectedStates.feature_Nl)
    log_cvx             = zeros(BUFFERSIZE,mpcParams.N,3,num_lap)
    log_cvy             = zeros(BUFFERSIZE,mpcParams.N,4,num_lap)
    log_cpsi            = zeros(BUFFERSIZE,mpcParams.N,3,num_lap)
    solHistory          = SolHistory(BUFFERSIZE,mpcParams.N,6,num_lap)
    selectHistory       = zeros(BUFFERSIZE,num_lap,selectedStates.Nl*selectedStates.Np,6)
    GP_vy_History       = zeros(BUFFERSIZE,num_lap,mpcParams.N)
    GP_psidot_History   = zeros(BUFFERSIZE,num_lap,mpcParams.N)
    selectFeatureHistory= zeros(BUFFERSIZE,num_lap,selectedStates.feature_Np,6)
    statusHistory       = Array{ASCIIString}(BUFFERSIZE,num_lap)

    mdl_pF           = MpcModel_pF(mpcParams_pF,modelParams)
    mdl_kin     	 = MpcModel_convhull_kin(mpcParams_4s,modelParams,selectedStates)
    mdl_convhull     = MpcModel_convhull_dyn_iden(mpcParams,modelParams,selectedStates)
    mdl_kin_lin      = MpcModel_convhull_kin_linear(mpcParams_4s,modelParams,selectedStates)
    mdl_dyn_lin      = MpcModel_convhull_dyn_linear(mpcParams,modelParams,selectedStates)

    if !PF_FLAG
        # FUNCTION DUMMY CALLS: this is important to call all the functions that will be used before for initial compiling
        data = load("$(homedir())/simulations/dummy_function/oldSS.jld") # oldSS.jld is a dummy data for initialization
        oldSS_dummy = data["oldSS"]
        lapStatus_dummy = LapStatus(1+max(selectedStates.feature_Nl,selectedStates.Nl),1,false,false,0.3)
        selectedStates_dummy=find_SS(oldSS_dummy,selectedStates,5.2,z_prev,lapStatus_dummy,modelParams,mpcParams,track)
        z = rand(1,6); u = rand(1,2)
        data = load("$(homedir())/simulations/dummy_function/FeatureDataCollecting.jld")
        feature_z = data["feature_z"]
        feature_u = data["feature_u"]
        (iden_z,iden_u)=find_feature_dist(feature_z,feature_u,z,u,selectedStates)
        data = load("$(homedir())/simulations/dummy_function/path_following.jld") # oldSS.jld is a dummy data for initialization
        solHistory_dummy = data["solHistory"]
        # (~,~,~)=find_feature_space(solHistory_dummy,2*ones(1,6),rand(1,2),lapStatus_dummy,selectedStates,posInfo)
        (c_Vx,c_Vy,c_Psi)=coeff_iden_dist(iden_z,iden_u)
        (~,~,~)=solveMpcProblem_convhull_dyn_iden(mdl_convhull,mpcSol,mpcCoeff,rand(6),rand(mpcParams.N+1,6),rand(mpcParams.N,2),selectedStates_dummy,track,rand(mpcParams.N),rand(mpcParams.N))
        (~,~,~)=solveMpcProblem_convhull_dyn_linear(mdl_dyn_lin,mpcSol,mpcParams,modelParams,lapStatus,rand(mpcParams.N+1,6),rand(mpcParams.N,2),rand(mpcParams.N+1,6),rand(mpcParams.N,2),selectedStates_dummy,track,rand(mpcParams.N),rand(mpcParams.N))
        selectedStates_dummy.selStates=s6_to_s4(selectedStates_dummy.selStates)
        (~,~,~)=solveMpcProblem_convhull_kin_linear(mdl_kin_lin,mpcSol,mpcParams_4s,modelParams,rand(mpcParams.N+1,4),rand(mpcParams.N,2),rand(mpcParams.N+1,4),rand(mpcParams.N,2),selectedStates_dummy,track)
        (~,~,~)=solveMpcProblem_convhull_kin(mdl_kin,mpcSol,rand(4),rand(mpcParams.N+1,4),rand(mpcParams.N,2),selectedStates_dummy,track,rand(mpcParams.N),rand(mpcParams.N))
        (~,~,~)=car_pre_dyn(rand(1,6)+1,rand(mpcParams.N,2),track,modelParams,6)
        (~,~,~)=find_SS_dist(solHistory,rand(1,6),rand(1,2),lapStatus,selectedStates)
    end

    if FEATURE_FLAG
        # FEATURE DATA READING
        if get_param("feature_flag")
	        data = load("$(homedir())/simulations/Feature_Data/FeatureDataCollecting.jld")
	    else
	        data = load("$(homedir())/experiments/Feature_Data/FeatureDataCollecting.jld")
	    end
        feature_z = data["feature_z"]
        feature_u = data["feature_u"]
        println("Number of total feature points from 8-shape:",size(feature_z,1))
    end

    if GP_LOCAL_FLAG || GP_FULL_FLAG
        # FEATURE DATA READING FOR GPR # THIS PART NEEDS TO BE COMMENTED OUT FOR THE FIRST TIME LMPC FOR GP DATA COLLECTING
        if PF_FLAG
            file_GP_name = "PF"
        else
            if LMPC_FLAG # CONTROLLER FLAG
                if TI_TV_FLAG
                    file_GP_name = "SYS_ID_TI"
                else
                    file_GP_name = "SYS_ID_TV"
                end
            elseif LMPC_DYN_FLAG
                file_GP_name = "DYN_LIN"
            else
                file_name = "SYS_ID_KIN_LIN"
                if TI_TV_FLAG
                    file_GP_name *= "_TI"
                else
                    file_GP_name *= "_TV"
                end
            end
        end
        if get_param("sim_flag")
	        data = load("$(homedir())/simulations/Feature_Data/FeatureData_GP-$(file_GP_name).jld")
	    else
	        data = load("$(homedir())/experiments/Feature_Data/FeatureData_GP-$(file_GP_name).jld")
	    end
        num_spare            = 30 # THE NUMBER OF POINTS SELECTED FOR SPARE GP
        feature_GP_z         = data["feature_GP_z"]
        feature_GP_u         = data["feature_GP_u"]
        feature_GP_vy_e      = data["feature_GP_vy_e"]
        feature_GP_psidot_e  = data["feature_GP_psidot_e"]
        feature_GP_z         = feature_GP_z[1:num_spare:end,:]
        feature_GP_u         = feature_GP_u[1:num_spare:end,:]
        feature_GP_vy_e      = feature_GP_vy_e[1:num_spare:end]
        feature_GP_psidot_e  = feature_GP_psidot_e[1:num_spare:end]

        GP_e_vy_prepare      = GP_prepare(feature_GP_vy_e,feature_GP_z,feature_GP_u)
        GP_e_psi_dot_prepare = GP_prepare(feature_GP_psidot_e,feature_GP_z,feature_GP_u)
        GP_feature           = hcat(feature_GP_z,feature_GP_u)

        # GP RELATED FUNCTION DUMMY CALLING
        # regre(rand(1,6),rand(1,2),feature_GP_vy_e,feature_GP_z,feature_GP_u)
        GP_full_vy(rand(1,6),rand(1,2),GP_feature,GP_e_vy_prepare)
        GP_full_psidot(rand(1,6),rand(1,2),GP_feature,GP_e_vy_prepare)
    else # THIS IS FOR COLLECTING FEATURE DATA FOR GPR
        feature_GP_z         = zeros(10000,6)
        feature_GP_u         = zeros(10000,2)
        feature_GP_vy_e      = zeros(10000)
        feature_GP_psidot_e  = zeros(10000)
        k = 1
    end
    
    if PF_FLAG
        lapStatus.currentLap = 1
    else
        lapStatus.currentLap = 1+max(selectedStates.feature_Nl,selectedStates.Nl)
        data  = load("$(homedir())/simulations/path_following.jld")
        oldSS = data["oldSS"]
        solHistory = data["solHistory"]
    end
    println(lapStatus)
    println("track maximum curvature: ",track.max_curvature)

    println("Finished LMPC NODE initialization.")

    while ! is_shutdown()
        if z_est[6] > 0    

            # CONTROL SIGNAL PUBLISHING
            cmd.header.stamp            = get_rostime()
            mpcSol_to_pub.header.stamp  = get_rostime()     
            publish(pub, cmd)
            publish(mpcSol_pub, mpcSol_to_pub)

            # LAP SWITCHING
            if lapStatus.nextLap
                println("Finishing one lap at iteration ",lapStatus.currentIt)
                if PF_FLAG || (!PF_FLAG && lapStatus.currentLap > 1+max(selectedStates.feature_Nl,selectedStates.Nl)) 
                    # IN CONSISTANT WITH THE DATA SAVING PART: AVOIDING SAVING THE DATA FOR THE FIRST WARM UP LAP IN LMPC
                    # SAFE SET COST UPDATE
                    oldSS.oldCost[lapStatus.currentLap]     = lapStatus.currentIt-1
                    solHistory.cost[lapStatus.currentLap]   = lapStatus.currentIt-1
                    oldSS.cost2target[:,lapStatus.currentLap] = lapStatus.currentIt - oldSS.cost2target[:,lapStatus.currentLap]
                end 

                lapStatus.nextLap = false

                setvalue(mdl_pF.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,1]-posInfo.s_target)
                setvalue(mdl_pF.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,1]-posInfo.s_target)
                setvalue(mdl_convhull.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,1]-posInfo.s_target)
                setvalue(mdl_convhull.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,1]-posInfo.s_target)

                if z_prev[1,1]>posInfo.s_target
                    z_prev[:,1] -= posInfo.s_target
                end

                lapStatus.currentLap += 1
                lapStatus.currentIt = 1

                if GP_FULL_FLAG && GP_HISTORY_FLAG # USING DATA FROM PREVIOUS LAPS TO DO FULL GPR
                    # CONSTRUCT GP_related from solHistory
                    GP_e_vy_prepare      = GP_prepare(feature_GP_vy_e,feature_GP_z,feature_GP_u)
                    GP_e_psi_dot_prepare = GP_prepare(feature_GP_psidot_e,feature_GP_z,feature_GP_u)
                    GP_feature           = hcat(feature_GP_z,feature_GP_u)
                end # THIS PART IS STILL NOT READY

                if lapStatus.currentLap > num_lap # to save the data at the end before error pops up
                    # DATA SAVING
                    run_time = Dates.format(now(),"yyyy-mm-dd-H:M")
                    log_path = "$(homedir())/$(folder_name)/LMPC-$(file_name)-$(run_time).jld"
                    save(log_path,"log_cvx",log_cvx,"log_cvy",log_cvy,"log_cpsi",log_cpsi,"GP_vy_History",GP_vy_History,"GP_psidot_History",GP_psidot_History,
				                  "oldTraj",oldTraj,"selectedStates",selectedStates,"oldSS",oldSS,"solHistory",solHistory,
				                  "selectHistory",selectHistory,"selectFeatureHistory",selectFeatureHistory,"statusHistory",statusHistory,
				                  "track",track,"modelParams",modelParams,"mpcParams",mpcParams)
                    # COLLECT ONE STEP PREDICTION ERROR FOR GPR
                    if !GP_LOCAL_FLAG && !GP_FULL_FLAG
                        run_time = Dates.format(now(),"yyyy-mm-dd-H:M")
                        if get_param("sim_flag")
                        	log_path = "$(homedir())/simulations/Feature_Data/FeatureData_GP-$(file_name).jld"
                        else
                        	log_path = "$(homedir())/experiments/Feature_Data/FeatureData_GP-$(file_name).jld"
                        end
                        feature_GP_z        = feature_GP_z[1:k-1,:]
				        feature_GP_u        = feature_GP_u[1:k-1,:]
				        feature_GP_vy_e     = feature_GP_vy_e[1:k-1]
				        feature_GP_psidot_e = feature_GP_psidot_e[1:k-1]
                        save(log_path,"feature_GP_z",feature_GP_z,"feature_GP_u",feature_GP_u,"feature_GP_vy_e",feature_GP_vy_e,"feature_GP_psidot_e",feature_GP_psidot_e) 
                    end
                end
            end

            # MPC CONTROLLER OPTIMIZATION
            if lapStatus.currentLap<=1+max(selectedStates.feature_Nl,selectedStates.Nl) # pF CONTROLLER
                # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
                z_curr = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
                # println("z_curr:",round(z_curr,2))
                z_kin = [z_est[6],z_est[5],z_est[4],sqrt(z_est[1]^2+z_est[2]^2)]
                # z_curr = xyFrame_to_trackFrame(z_true,track)
                # z_kin = [z_curr[1],z_curr[2],z_curr[3],sqrt(z_curr[4]^2+z_curr[5]^2)]
                # println("         state from LMPC nodes",round(mpcSol.z[2,:],2))
                # println("estimate state from LMPC nodes",round(z_curr,2))
                
                (mpcSol.z,mpcSol.u,sol_status) = solveMpcProblem_pathFollow(mdl_pF,mpcParams_pF,modelParams,mpcSol,z_kin,z_prev,u_prev,track)
                
            else # LMPC CONTROLLER
                # FOR QUICK LMPC STARTING, the next time, change the path following lap number to 1 and change the initial lapStatus to selectedStates.
                if PF_FLAG
                    # save("$(homedir())/simulations/path_following.jld","oldSS",oldSS,"oldSS_true",oldSS_true,"solHistory",solHistory)
                    if get_param("sim_flag")
                    	save("$(homedir())/simulations/path_following.jld","oldSS",oldSS,"solHistory",solHistory)
				    else
                    	save("$(homedir())/experiments/path_following.jld","oldSS",oldSS,"solHistory",solHistory)
				    end
                end
                
                # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
                # z_curr = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
                z_curr = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
                # z_curr = xyFrame_to_trackFrame(z_true,track)

                # println("z_curr from LMPC node",z_curr)
                
                if LMPC_FLAG
                    ######################################################################
                    ############### CHOICE 1: DO MPC ON SYS_ID MODEL #####################
                    ######################################################################
                    tic()
                    # PREPARE THE PREVIOUS SOLUTION FOR LMPC, WHEN SWITCHING FROM pF TO LMPC
                    if size(z_prev,2)==4
                        z_prev = hcat(z_prev,zeros(size(z_prev,1),2))
                    end
                    # println("z_prev is:",round(z_prev,3))
                    # SYS_ID
                    if !TI_TV_FLAG
                        # TV SYS_ID                
                        z_to_iden = vcat(z_curr',z_prev[3:end,:])
                        u_to_iden = vcat(u_prev[2:end,:],u_prev[end,:])
                        for i in 1:mpcParams.N
                            if FEATURE_FLAG
                                # SELECTING THE FEATURE POINTS FROM DATASET
                                (iden_z,iden_u,z_iden_plot)=find_feature_dist(feature_z,feature_u,z_to_iden[i,:],u_to_iden[i,:],selectedStates)
                            else
                                if NORM_SPACE_FLAG
                                    # SELECTING THE FEATURE POINTS FROM HISTORY BASED ON 2-NORM CRITERION
                                    (iden_z,iden_u,z_iden_plot)=find_SS_dist(solHistory,z_to_iden[i,:],u_to_iden[i,:],lapStatus,selectedStates)
                                else
                                    # SELECTING THE FEATURE POINTS FROM HISTORY BASED ON SPATIAL CRITERION
                                    (iden_z,iden_u,z_iden_plot)=find_feature_space(solHistory,z_to_iden[i,:],u_to_iden[i,:],lapStatus,selectedStates,posInfo)
                                end
                            end
                            (mpcCoeff.c_Vx[i,:],mpcCoeff.c_Vy[i,:],mpcCoeff.c_Psi[i,:])=coeff_iden_dist(iden_z,iden_u)
                        end
                    else
                        # TI SYS_ID
                        if FEATURE_FLAG
                            # SELECTING THE FEATURE POINTS FROM DATASET
                            (iden_z,iden_u,z_iden_plot)=find_feature_dist(feature_z,feature_u,z_curr',u_prev[2,:],selectedStates)
                        else
                            if NORM_SPACE_FLAG
                                # SELECTING THE FEATURE POINTS FROM HISTORY
                                (iden_z,iden_u,z_iden_plot)=find_SS_dist(solHistory,z_curr',u_prev[2,:],lapStatus,selectedStates)
                            else
                                (iden_z,iden_u,z_iden_plot)=find_feature_space(solHistory,z_curr',u_prev[2,:],lapStatus,selectedStates,posInfo)
                            end
                        end
                        (mpcCoeff.c_Vx[1,:],mpcCoeff.c_Vy[1,:],mpcCoeff.c_Psi[1,:])=coeff_iden_dist(iden_z,iden_u)
                        for i in 2:mpcParams.N
                            mpcCoeff.c_Vx[i,:]=mpcCoeff.c_Vx[1,:]
                            mpcCoeff.c_Vy[i,:]=mpcCoeff.c_Vy[1,:]
                            mpcCoeff.c_Psi[i,:]=mpcCoeff.c_Psi[1,:]
                        end
                        # println("c_Vx",round(mpcCoeff.c_Vx[1,:],2))
                        # println("c_Vy",round(mpcCoeff.c_Vy[1,:],2))
                        # println("c_Psi",round(mpcCoeff.c_Psi[1,:],2))
                    end
                    # GPR DISTURBANCE ID
                    GP_e_vy     = zeros(mpcParams.N)
                    GP_e_psidot = zeros(mpcParams.N)
                    z_to_iden = vcat(z_curr',z_prev[3:end,:])
                    u_to_iden = vcat(u_prev[2:end,:],u_prev[end,:])
                    for i = 1:mpcParams.N
                        if GP_LOCAL_FLAG
                            GP_e_vy[i]      = regre(z_to_iden[i,:],u_to_iden[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                            GP_e_psidot[i]  = regre(z_to_iden[i,:],u_to_iden[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                        elseif GP_FULL_FLAG
                            GP_e_vy[i]      = GP_full_vy(z_to_iden[i,:],u_to_iden[i,:],GP_feature,GP_e_vy_prepare)
                            GP_e_psidot[i]  = GP_full_psidot(z_to_iden[i,:],u_to_iden[i,:],GP_feature,GP_e_psi_dot_prepare)
                        else
                            GP_e_vy[i]      = 0
                            GP_e_psidot[i]  = 0
                        end
                    end
                    # println(GP_e_vy)
                    # println(GP_e_psidot)

                    # FEATURE POINTS VISUALIZATION
                    if FEATURE_FLAG
                        (z_iden_x, z_iden_y) = trackFrame_to_xyFrame(z_iden_plot,track_f)
                    else
                        (z_iden_x, z_iden_y) = trackFrame_to_xyFrame(z_iden_plot,track)
                    end
                    mpcSol_to_pub.z_iden_x = z_iden_x
                    mpcSol_to_pub.z_iden_y = z_iden_y
                    toc() # TIME FOR PREPARATION
                    # SAFESET POINT SELECTION
                    selectedStates=find_SS(oldSS,selectedStates,z_curr[1],z_prev,lapStatus,modelParams,mpcParams,track)
                    # println(selectedStates)
                    tic()
                    (mpcSol.z,mpcSol.u,sol_status)=solveMpcProblem_convhull_dyn_iden(mdl_convhull,mpcSol,mpcCoeff,z_curr,z_prev,u_prev,selectedStates,track,GP_e_vy,GP_e_psidot)
                    toc() # TIME FOR OPTIMIZATION
                elseif LMPC_DYN_FLAG # USING LINEARIZED DYNAMIC MODEL FOR LMPC, 6 STATES LMPC CONTROLLER 
                    ######################################################################
                    ############### CHOICE 2: DO MPC ON DYN_LIN MODEL ####################
                    ######################################################################
                    tic()
                    # PREPARE THE PREVIOUS SOLUTION FOR LMPC, WHEN SWITCHING FROM pF TO LMPC
                    if size(z_prev,2)==4
                        z_prev = hcat(z_prev,zeros(size(z_prev,1),2)) 
                    end
                    u_linear = vcat(u_prev[2:end,:],u_prev[end,:])
                    (z_linear,~,~) = car_pre_dyn(z_curr,u_linear,track,modelParams,6)
                    
                    # SAFESET POINT SELECTION
                    # println(z_curr)
                    selectedStates=find_SS(oldSS,selectedStates,z_est[6],z_prev,lapStatus,modelParams,mpcParams,track)
                    # println(selectedStates)

                    # GPR DISTURBANCE ID
                    for i = 1:mpcParams.N
                        if GP_LOCAL_FLAG
                            GP_e_vy[i]      = regre(z_linear[i,:],u_linear[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                            GP_e_psidot[i]  = regre(z_linear[i,:],u_linear[i,:],feature_GP_psidot_e,feature_GP_z,feature_GP_u)
                        elseif GP_FULL_FLAG
                            GP_e_vy[i]      = GP_full_vy(z_linear[i,:],u_linear[i,:],GP_feature,GP_e_vy_prepare)
                            GP_e_psidot[i]  = GP_full_psidot(z_linear[i,:],u_linear[i,:],GP_feature,GP_e_psi_dot_prepare)
                        else
                            GP_e_vy[i]      = 0
                            GP_e_psidot[i]  = 0
                        end
                    end
                    toc()
                    # println(GP_e_vy)
                    # println(GP_e_psidot)
                    tic()
                    (mpcSol.z,mpcSol.u,sol_status)=solveMpcProblem_convhull_dyn_linear(mdl_dyn_lin,mpcSol,mpcParams,modelParams,lapStatus,z_linear,u_linear,z_prev,u_prev,selectedStates,track,GP_e_vy,GP_e_psidot)
                    toc()

                elseif LMPC_KIN_FLAG
                	z_curr = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
	                z_kin = [z_est[6],z_est[5],z_est[4],sqrt(z_est[1]^2+z_est[2]^2)]
	                GP_e_vy     = zeros(mpcParams.N)
                    GP_e_psidot = zeros(mpcParams.N)
                    z_to_iden = vcat(z_kin',z_prev[3:end,:])
                    u_to_iden = vcat(u_prev[2:end,:],u_prev[end,:])
                    for i = 1:mpcParams.N
                        if GP_LOCAL_FLAG
                            GP_e_vy[i]      = regre(z_to_iden[i,:],u_to_iden[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                            GP_e_psidot[i]  = regre(z_to_iden[i,:],u_to_iden[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                        elseif GP_FULL_FLAG
                            GP_e_vy[i]      = GP_full_vy(z_to_iden[i,:],u_to_iden[i,:],GP_feature,GP_e_vy_prepare)
                            GP_e_psidot[i]  = GP_full_psidot(z_to_iden[i,:],u_to_iden[i,:],GP_feature,GP_e_psi_dot_prepare)
                        else
                            GP_e_vy[i]      = 0
                            GP_e_psidot[i]  = 0
                        end
                    end
                    # SAFESET SELECTION
                    selectedStates=find_SS(oldSS,selectedStates,z_curr[1],z_prev,lapStatus,modelParams,mpcParams_4s,track)
                    selectedStates.selStates=s6_to_s4(selectedStates.selStates)
                	(mpcSol.z,mpcSol.u,sol_status) = solveMpcProblem_convhull_kin(mdl_kin,mpcSol,z_kin,z_prev,u_prev,selectedStates,track,GP_e_vy,GP_e_psidot)
                else 
                    ######################################################################
                    ############### CHOICE 3: DO MPC ON KIN_LIN MODEL ####################
                    ######################################################################
                    tic()
                    z_linear        = zeros(mpcParams_4s.N+1,4)
                    z_linear[1,:]   = s6_to_s4(z_curr')
                    u_linear        = vcat(u_prev[2:end,:],u_prev[end,:])
                    z_dummy         = copy(z_curr) # 6-dimension state is needed for forecasting forward
                    
                    if !TI_TV_FLAG
                        # TV SYS_ID
                        for i=1:size(z_linear,1)-1 
                            # LMS SYS ID
                            if FEATURE_FLAG
                                (iden_z,iden_u,z_iden_plot)=find_feature_dist(feature_z,feature_u,z_dummy',u_linear[i,:],selectedStates)
                            else
                                if NORM_SPACE_FLAG
                                    # SELECTING THE FEATURE POINTS FROM HISTORY BASED ON 2-NORM CRITERION
                                    (iden_z,iden_u,z_iden_plot)=find_SS_dist(solHistory,z_dummy',u_linear[i,:],lapStatus,selectedStates)
                                else
                                    # SELECTING THE FEATURE POINTS FROM HISTORY BASED ON SPATIAL CRITERION
                                    (iden_z,iden_u,z_iden_plot)=find_feature_space(solHistory,z_dummy',u_linear[i,:],lapStatus,selectedStates,posInfo)
                                end
                            end
                            (mpcCoeff_dummy.c_Vx[1,:],mpcCoeff_dummy.c_Vy[1,:],mpcCoeff_dummy.c_Psi[1,:])=coeff_iden_dist(iden_z,iden_u)

                            mpcCoeff.c_Vx[i,:]=mpcCoeff_dummy.c_Vx[1,:]
                            mpcCoeff.c_Vy[i,:]=mpcCoeff_dummy.c_Vy[1,:]
                            mpcCoeff.c_Psi[i,:]=mpcCoeff_dummy.c_Psi[1,:]
                        
                            z_dummy=car_sim_iden_tv(z_dummy,u_linear[i,:],0.1,mpcCoeff_dummy,modelParams,track)
                            # GPR DISTURBANCE ID
                            if GP_LOCAL_FLAG
                                GP_e_vy      = regre(z_dummy,u_linear[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                                GP_e_psidot  = regre(z_dummy,u_linear[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                            elseif GP_FULL_FLAG
                                GP_e_vy      = GP_full_vy(z_dummy,u_linear[i,:],GP_feature,GP_e_vy_prepare)
                                GP_e_psidot  = GP_full_psidot(z_dummy,u_linear[i,:],GP_feature,GP_e_psi_dot_prepare)
                            else
                                GP_e_vy      = 0
                                GP_e_psidot  = 0
                            end

                            z_dummy[5] += GP_e_vy
                            z_dummy[6] += GP_e_psidot
                            z_linear[i+1,:]=s6_to_s4(z_dummy')
                        end
                    else
                        # TI SYS_ID
                        if FEATURE_FLAG # LMS SYS ID
                            (iden_z,iden_u,z_iden_plot)=find_feature_dist(feature_z,feature_u,z_dummy',u_linear[1,:],selectedStates)
                        else
                            if NORM_SPACE_FLAG
                                # SELECTING THE FEATURE POINTS FROM HISTORY BASED ON 2-NORM CRITERION
                                (iden_z,iden_u,z_iden_plot)=find_SS_dist(solHistory,z_dummy',u_linear[1,:],lapStatus,selectedStates)
                            else
                                # SELECTING THE FEATURE POINTS FROM HISTORY BASED ON SPATIAL CRITERION
                                (iden_z,iden_u,z_iden_plot)=find_feature_space(solHistory,z_dummy',u_linear[1,:],lapStatus,selectedStates,posInfo)
                            end
                        end
                        (mpcCoeff_dummy.c_Vx[1,:],mpcCoeff_dummy.c_Vy[1,:],mpcCoeff_dummy.c_Psi[1,:])=coeff_iden_dist(iden_z,iden_u)
                        
                        for i = 1:size(mpcCoeff.c_Vx,1)
                            mpcCoeff.c_Vx[i,:]=mpcCoeff_dummy.c_Vx[1,:]
                            mpcCoeff.c_Vy[i,:]=mpcCoeff_dummy.c_Vy[1,:]
                            mpcCoeff.c_Psi[i,:]=mpcCoeff_dummy.c_Psi[1,:]
                        end

                        for i=1:size(z_linear,1)-1 # GPR DISTURBANCE ID
                            if GP_LOCAL_FLAG
                                GP_e_vy      = regre(z_dummy,u_linear[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                                GP_e_psidot  = regre(z_dummy,u_linear[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                            elseif GP_FULL_FLAG
                                GP_e_vy      = GP_full_vy(z_dummy,u_linear[i,:],GP_feature,GP_e_vy_prepare)
                                GP_e_psidot  = GP_full_psidot(z_dummy,u_linear[i,:],GP_feature,GP_e_psi_dot_prepare)
                            else
                                GP_e_vy      = 0
                                GP_e_psidot  = 0
                            end
                            # THRESHOLDING
                            GP_e_vy      = min(0.025,GP_e_vy)
                            GP_e_vy      = max(-0.025,GP_e_vy)
                            GP_e_psidot = min(0.1,GP_e_psidot)
                            GP_e_psidot = max(-0.1,GP_e_psidot)
                            # println("z_dummy",round(z_dummy,2))
                            z_dummy=car_sim_iden_tv(z_dummy,u_linear[i,:],0.1,mpcCoeff_dummy,modelParams,track)
                            # println("GP_e_vy",GP_e_vy)
                            z_dummy[5] += GP_e_vy[1]
                            z_dummy[6] += GP_e_psidot[1]
                            z_linear[i+1,:] = s6_to_s4(z_dummy')
                        end
                    end # end of TI/TV

                    # FEATURE POINTS VISUALIZATION
                    if FEATURE_FLAG
                        (z_iden_x, z_iden_y) = trackFrame_to_xyFrame(z_iden_plot,track_f)
                    else
                        (z_iden_x, z_iden_y) = trackFrame_to_xyFrame(z_iden_plot,track)
                    end
                    mpcSol_to_pub.z_iden_x = z_iden_x
                    mpcSol_to_pub.z_iden_y = z_iden_y

                    # SAFESET SELECTION
                    selectedStates=find_SS(oldSS,selectedStates,z_curr[1],z_prev,lapStatus,modelParams,mpcParams_4s,track)
                    selectedStates.selStates=s6_to_s4(selectedStates.selStates)
                    toc()
                    
                    tic()
                    (mpcSol.z,mpcSol.u,sol_status)=solveMpcProblem_convhull_kin_linear(mdl_kin_lin,mpcSol,mpcParams_4s,modelParams,z_linear,u_linear,z_prev,u_prev,selectedStates,track)
                    toc()
                end # end of IF:IDEN_MODEL/DYN_LIN_MODEL/IDEN_KIN_LIN_MODEL

                # COLLECT ONE STEP PREDICTION ERROR FOR GPR 
                if !GP_LOCAL_FLAG && !GP_FULL_FLAG && lapStatus.currentIt>1
                    feature_GP_z[k,:]       = solHistory.z[lapStatus.currentIt-1,lapStatus.currentLap,1,:]
                    feature_GP_u[k,:]       = u_prev[1,:]
                    feature_GP_vy_e[k]      = z_curr[5]-solHistory.z[lapStatus.currentIt-1,lapStatus.currentLap,2,5]
                    feature_GP_psidot_e[k]  = z_curr[6]-solHistory.z[lapStatus.currentIt-1,lapStatus.currentLap,2,6]
                    k += 1 
                end
                
                # BACK-UP FOR POSSIBLE NON-OPTIMAL SOLUTION
                sol_status_dummy = "$sol_status"
                if sol_status_dummy[1] != 'O'
                    mpcSol.u=copy(u_prev)
                    if LMPC_FLAG || LMPC_DYN_FLAG
                        (mpcSol.z,~,~)=car_pre_dyn(z_curr,mpcSol.u,track,modelParams,6)
                    else
                        (mpcSol.z,~,~)=car_pre_dyn(z_curr,mpcSol.u,track,modelParams,4)
                    end
                end 
            end # end of IF:pF/LMPC
            # tic()
            # DATA WRITING AND COUNTER UPDATE
            log_cvx[lapStatus.currentIt,:,:,lapStatus.currentLap]   = mpcCoeff.c_Vx       
            log_cvy[lapStatus.currentIt,:,:,lapStatus.currentLap]   = mpcCoeff.c_Vy       
            log_cpsi[lapStatus.currentIt,:,:,lapStatus.currentLap]  = mpcCoeff.c_Psi

            n_state = size(mpcSol.z,2)

            mpcSol.a_x = mpcSol.u[1+mpcParams.delay_a,1] 
            mpcSol.d_f = mpcSol.u[1+mpcParams.delay_df,2]

            # mpcSol.a_x = mpcSol.u[1,1]
            # mpcSol.d_f = mpcSol.u[1,2]
            if length(mpcSol.df_his)==1
                mpcSol.df_his[1] = mpcSol.u[1+mpcParams.delay_df,2]
            else
                # INPUT DELAY HISTORY UPDATE
                mpcSol.df_his[1:end-1] = mpcSol.df_his[2:end]
                mpcSol.df_his[end] = mpcSol.u[1+mpcParams.delay_df,2]
            end

            if length(mpcSol.a_his)==1
                mpcSol.a_his[1] = mpcSol.u[1+mpcParams.delay_a,1]
            else
                # INPUT DELAY HISTORY UPDATE
                mpcSol.a_his[1:end-1] = mpcSol.a_his[2:end]
                mpcSol.a_his[end] = mpcSol.u[1+mpcParams.delay_a,1]
            end
            
            if PF_FLAG || (!PF_FLAG && lapStatus.currentLap > 1+max(selectedStates.feature_Nl,selectedStates.Nl))
                # println("saving history data")
                solHistory.z[lapStatus.currentIt,lapStatus.currentLap,:,1:n_state]=mpcSol.z
                solHistory.z[lapStatus.currentIt,lapStatus.currentLap,1,4:6]=z_curr[4:6]  # [z_est[1],z_est[2],z_est[3]] # THIS LINE IS REALLY IMPORTANT FOR SYS_ID FROM pF
                # solHistory.z[lapStatus.currentIt,lapStatus.currentLap,1,4:6]=[z_true[3],z_true[4],z_true[6]] # THIS LINE IS REALLY IMPORTANT FOR SYS_ID FROM pF
                solHistory.u[lapStatus.currentIt,lapStatus.currentLap,:,:]=mpcSol.u

                # SAFESET DATA SAVING BASED ON CONTROLLER'S FREQUENCY
                oldSS.oldSS[lapStatus.currentIt,:,lapStatus.currentLap]=z_curr # [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
                # oldSS_true.oldSS[lapStatus.currentIt,:,lapStatus.currentLap]=xyFrame_to_trackFrame(z_true,track)
                oldSS.cost2target[lapStatus.currentIt,lapStatus.currentLap]=lapStatus.currentIt
            end

            statusHistory[lapStatus.currentIt,lapStatus.currentLap] = "$sol_status"
            if !LMPC_DYN_FLAG && !LMPC_KIN_FLAG && !PF_FLAG && lapStatus.currentLap > 1+max(selectedStates.feature_Nl,selectedStates.Nl) 
                selectFeatureHistory[lapStatus.currentIt,lapStatus.currentLap,:,:] = z_iden_plot
            end
            if (!PF_FLAG && lapStatus.currentLap > 1+max(selectedStates.feature_Nl,selectedStates.Nl))
                if !LMPC_FLAG && !LMPC_DYN_FLAG
                    selectHistory[lapStatus.currentIt,lapStatus.currentLap,:,1:4] = selectedStates.selStates
                else
                    selectHistory[lapStatus.currentIt,lapStatus.currentLap,:,:] = selectedStates.selStates
                end
            end
            if !GP_LOCAL_FLAG || !GP_FULL_FLAG
                GP_vy_History[lapStatus.currentIt,lapStatus.currentLap,:] = GP_e_vy
                GP_psidot_History[lapStatus.currentIt,lapStatus.currentLap,:] = GP_e_psidot
            end

            # VISUALIZATION COORDINATE CALCULATION FOR view_trajectory.jl NODE
            (z_x,z_y) = trackFrame_to_xyFrame(mpcSol.z,track)
            mpcSol_to_pub.z_x = z_x
            mpcSol_to_pub.z_y = z_y
            # println(selectedStates.selStates)
            (SS_x,SS_y) = trackFrame_to_xyFrame(selectedStates.selStates,track)
            mpcSol_to_pub.SS_x = SS_x
            mpcSol_to_pub.SS_y = SS_y
            mpcSol_to_pub.z_vx = mpcSol.z[:,4]
            mpcSol_to_pub.SS_vx = selectedStates.selStates[:,4]
            mpcSol_to_pub.z_s = mpcSol.z[:,1]
            mpcSol_to_pub.SS_s = selectedStates.selStates[:,1]
            # FORECASTING POINTS FROM THE DYNAMIC MODEL
            if length(z_curr)==6
                (z_fore,~,~) = car_pre_dyn_true(z_curr,mpcSol.u,track,modelParams,6)
                (z_fore_x,z_fore_y) = trackFrame_to_xyFrame(z_fore,track)
                mpcSol_to_pub.z_fore_x = z_fore_x
                mpcSol_to_pub.z_fore_y = z_fore_y
            end

            cmd.servo   = convert(Float32,mpcSol.d_f)
            cmd.motor   = convert(Float32,mpcSol.a_x)

            # cmd.servo   = convert(Float32,-0.2)
            # cmd.motor   = convert(Float32,0.0)


            z_prev      = copy(mpcSol.z)
            u_prev      = copy(mpcSol.u)
            # toc()
            println("$sol_status Current Lap: ", lapStatus.currentLap, ", It: ", lapStatus.currentIt, " v: $(z_est[1])")
            lapStatus.currentIt += 1
        else
            println("No estimation data received!")
        end
        rossleep(loop_rate)
    end # END OF THE WHILE LOOP
    # THIS IS FOR THE LAST NO FINISHED LAP
    solHistory.cost[lapStatus.currentLap]   = lapStatus.currentIt-1
    # DATA SAVING
    run_time = Dates.format(now(),"yyyy-mm-dd-H:M")
    log_path = "$(homedir())/$(folder_name)/LMPC-$(file_name)-$(run_time).jld"
    save(log_path,"log_cvx",log_cvx,"log_cvy",log_cvy,"log_cpsi",log_cpsi,"GP_vy_History",GP_vy_History,"GP_psidot_History",GP_psidot_History,
                  "oldTraj",oldTraj,"selectedStates",selectedStates,"oldSS",oldSS,"solHistory",solHistory,
                  "selectHistory",selectHistory,"selectFeatureHistory",selectFeatureHistory,"statusHistory",statusHistory,
                  "track",track,"modelParams",modelParams,"mpcParams",mpcParams)
    # COLLECT ONE STEP PREDICTION ERROR FOR GPR 
    if !GP_LOCAL_FLAG && !GP_FULL_FLAG
        run_time = Dates.format(now(),"yyyy-mm-dd-H:M")
        if get_param("sim_flag")
	        log_path = "$(homedir())/simulations/Feature_Data/FeatureData_GP-$(file_name).jld"
	    else
	        log_path = "$(homedir())/experiments/Feature_Data/FeatureData_GP-$(file_name).jld"
	    end
        # CUT THE FRONT AND REAR TAIL BEFORE SAVING THE DATA
        feature_GP_z        = feature_GP_z[1:k-1,:]
        feature_GP_u        = feature_GP_u[1:k-1,:]
        feature_GP_vy_e     = feature_GP_vy_e[1:k-1]
        feature_GP_psidot_e = feature_GP_psidot_e[1:k-1]
        save(log_path,"feature_GP_z",feature_GP_z,"feature_GP_u",feature_GP_u,
                      "feature_GP_vy_e",feature_GP_vy_e,"track",track,
                      "feature_GP_psidot_e",feature_GP_psidot_e) 
    end
    println("Exiting LMPC node. Saved data to $log_path.")
end

if ! isinteractive()
    main()
end

# zCurr[1] = v_x
# zCurr[2] = v_y
# zCurr[3] = psiDot
# zCurr[4] = ePsi
# zCurr[5] = eY
# zCurr[6] = s
