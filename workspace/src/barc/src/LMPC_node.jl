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
include("barc_lib/LMPC/functions.jl")
include("barc_lib/LMPC/MPC_models.jl")
include("barc_lib/LMPC/coeffConstraintCost.jl")
include("barc_lib/LMPC/solveMpcProblem.jl")
include("barc_lib/simModel.jl")

function SE_callback(msg::pos_info,acc_f::Array{Float64},lapStatus::LapStatus,posInfo::PosInfo,mpcSol::MpcSol,z_est::Array{Float64,1},x_est::Array{Float64,1})
    z_est[:]                  = [msg.v_x,msg.v_y,msg.psiDot,msg.epsi,msg.ey,msg.s,acc_f[1]] # the last variable is filtered acceleration
    x_est[:]                  = [msg.x,msg.y,msg.psi,msg.v]  
    
    if z_est[6] <= lapStatus.s_lapTrigger && lapStatus.switchLap
        lapStatus.nextLap = true
        lapStatus.switchLap = false
    elseif z_est[6] > lapStatus.s_lapTrigger
        lapStatus.switchLap = true
    end
end

# This is the main function, it is called when the node is started.
function main()
    println("Starting LMPC node.")

    const BUFFERSIZE       = 500
    const LMPC_LAP         = 10
    const PF_FLAG          = true

    track_data       = createTrack("MSC_lab")
    track            = Track(track_data)
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
    z_est            = zeros(7)          # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
    x_est            = zeros(4)          # (x, y, psi, v)
    cmd              = ECU()             # CONTROL SIGNAL MESSAGE INITIALIZATION
    mpcSol_to_pub    = mpc_solution()    # MPC SOLUTION PUBLISHING MESSAGE INITIALIZATION

    InitializeParameters(mpcSol,mpcParams,mpcParams_4s,mpcParams_pF,modelParams,posInfo,mpcCoeff,mpcCoeff_dummy,BUFFERSIZE,selectedStates,oldSS,LMPC_LAP)
    z_prev              = mpcSol.z
    u_prev              = mpcSol.u
  
    num_lap             = LMPC_LAP+1+selectedStates.Nl
    log_cvx             = zeros(BUFFERSIZE,mpcParams.N,3,num_lap)
    log_cvy             = zeros(BUFFERSIZE,mpcParams.N,4,num_lap)
    log_cpsi            = zeros(BUFFERSIZE,mpcParams.N,3,num_lap)
    solHistory          = SolHistory(BUFFERSIZE,mpcParams.N,6,num_lap)
    
    mdl_pF           = MpcModel_pF(mpcParams_pF,modelParams)
    mdl_convhull     = MpcModel_convhull_dyn_iden(mpcParams,modelParams,selectedStates)
    mdl_kin_lin      = MpcModel_convhull_kin_linear(mpcParams_4s,modelParams,selectedStates)

    # FEATURE DATA READING
    data = load("$(homedir())/simulations/Feature_Data/FeatureDataCollecting.jld")
    feature_z = data["feature_z"]
    feature_u = data["feature_u"]

    # FEATURE DATA READING FOR GPR # THIS PART NEEDS TO BE COMMENTED OUT FOR THE FIRST TIME LMPC FOR GP DATA COLLECTING
    # data = load("$(homedir())/simulations/Feature_Data/FeatureData_GP.jld")
    # num_spare            = 30 # THE NUMBER OF POINTS SELECTED FOR SPARE GP
    # feature_GP_z         = data["feature_GP_z"]
    # feature_GP_u         = data["feature_GP_u"]
    # feature_GP_vy_e      = data["feature_GP_vy_e"]
    # feature_GP_psidot_e  = data["feature_GP_psidot_e"]
    # feature_GP_z         = feature_GP_z[1:num_spare:end,:]
    # feature_GP_u         = feature_GP_u[1:num_spare:end,:]
    # feature_GP_vy_e      = feature_GP_vy_e[1:num_spare:end]
    # feature_GP_psidot_e  = feature_GP_psidot_e[1:num_spare:end]

    # GP_e_vy_prepare      = GP_prepare(feature_GP_vy_e,feature_GP_z,feature_GP_u)
    # GP_e_psi_dot_prepare = GP_prepare(feature_GP_psidot_e,feature_GP_z,feature_GP_u)
    # GP_feature           = hcat(feature_GP_z,feature_GP_u)

    # NODE INITIALIZATION
    init_node("mpc_traj")
    loop_rate   = Rate(1/modelParams.dt)
    pub         = Publisher("ecu", ECU, queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
    mpcSol_pub  = Publisher("mpc_solution", mpc_solution, queue_size=1)::RobotOS.Publisher{barc.msg.mpc_solution}; acc_f = [0.0]
    s1          = Subscriber("pos_info", pos_info, SE_callback, (acc_f,lapStatus,posInfo,mpcSol,oldTraj,z_est,x_est),queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
    s2          = Subscriber("real_val", pos_info, ST_callback, (z_true,), queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}

    println("Finished LMPC NODE initialization.")
    # PRECOMPILE THE FUNCTIONS NEEDED
    include("functions_dummy_calls.jl")
    
    if !PF_FLAG
        data  = load("$(homedir())/simulations/path_following.jld")
        oldSS = data["oldSS"]
    end

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
                lapSwitchUpdate(oldSS,lapStatus,solHistory,mdl_pF,mdl_convhull,z_prev)
            end

            # OPTIMIZATION
            println("$sol_status Current Lap: ", lapStatus.currentLap, ", It: ", lapStatus.currentIt, " v: $(z_est[1])")
            if lapStatus.currentLap<=1+selectedStates.Nl
                # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
                z_curr = [z_est[6],z_est[5],z_est[4],sqrt(z_est[1]^2+z_est[2]^2)]
                (mpcSol.z,mpcSol.u,sol_status) = solveMpcProblem_pathFollow(mdl_pF,mpcParams_pF,modelParams,mpcSol,z_curr,z_prev,u_prev,track)
                z_prev      = copy(mpcSol.z)
                u_prev      = copy(mpcSol.u)
            else
                # FOR QUICK LMPC STARTING, the next time, change the path following lap number to 1 and change the initial lapStatus to selectedStates.
                save("$(homedir())/simulations/path_following.jld","oldSS",oldSS)
                tic()
                # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
                z_curr = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
                
                ######################################################################
                ############### CHOICE 1: DO MPC ON SYS_ID MODEL #####################
                ######################################################################
                # SAFESET POINT SELECTION
                selectedStates=find_SS(oldSS,selectedStates,z_prev,lapStatus,modelParams,mpcParams,track)
                # PREPARE THE PREVIOUS SOLUTION FOR LMPC
                if size(z_prev,2)==4
                    z_prev = hcat(z_prev,zeros(size(z_prev,1),2))
                end
        
                # # TV SYS_ID                
                # z_to_iden = vcat(z_curr',z_prev[3:end,:])
                # u_to_iden = vcat(u_prev[2:end,:],u_prev[end,:])
                # for i in 1:mpcParams.N
                #     z = z_to_iden[i,:]
                #     u = u_to_iden[i,:]
                #     (iden_z,iden_u)=find_feature_dist(feature_z,feature_u,z,u)
                #     (mpcCoeff.c_Vx[i,:],mpcCoeff.c_Vy[i,:],mpcCoeff.c_Psi[i,:])=coeff_iden_dist(iden_z,iden_u)
                # end

                # TI SYS_ID
                z_to_iden = vcat(z_curr',z_prev[3:end,:])
                u_to_iden = vcat(u_prev[2:end,:],u_prev[end,:])
                z = z_to_iden[1,:]
                u = u_to_iden[1,:]
                # println("input from LMPC node",u)
                # u[2] = mpcSol.df_his[1]

                # SELECTING THE FEATURE POINTS FROM DATASET
                (iden_z,iden_u,z_iden_plot)=find_feature_dist(feature_z,feature_u,z,u)
                # println(z_curr)
                # println(hcat(iden_z[:,:,1],iden_u))
                # # SELECTING THE FEATURE POINTS FROM HISTORY
                # (iden_z,iden_u,z_iden_plot)=find_SS_dist(solHistory,z_curr',u_prev[2,:],lapStatus)

                (z_iden_x, z_iden_y) = trackFrame_to_xyFrame(z_iden_plot,track)
                mpcSol_to_pub.z_iden_x = z_iden_x
                mpcSol_to_pub.z_iden_y = z_iden_y
                
                (mpcCoeff.c_Vx[1,:],mpcCoeff.c_Vy[1,:],mpcCoeff.c_Psi[1,:])=coeff_iden_dist(iden_z,iden_u)
                for i in 2:mpcParams.N
                    mpcCoeff.c_Vx[i,:]=mpcCoeff.c_Vx[1,:]
                    mpcCoeff.c_Vy[i,:]=mpcCoeff.c_Vy[1,:]
                    mpcCoeff.c_Psi[i,:]=mpcCoeff.c_Psi[1,:]
                end
                toc()
                tic()
                (mpcSol.z,mpcSol.u,sol_status)=solveMpcProblem_convhull_dyn_iden(mdl_convhull,mpcParams,mpcSol,mpcCoeff,lapStatus,z_curr,z_prev,u_prev,selectedStates,track)
                toc()
                ######################################################################
                ############### CHOICE 1: DO MPC ON KIN_LIN MODEL ####################
                ######################################################################
                # z_linear=zeros(mpcParams_4s.N+1,4)
                # z_linear[1,:]=s6_to_s4(z_curr')
                # u_linear=vcat(u_prev[2:end,:],u_prev[end,:])
                # z_dummy=copy(z_curr) # 6-dimension state is needed for forecasting forward
                
                # # TV SYS_ID
                # # for i=1:size(z_linear,1)-1
                # #     (iden_z,iden_u,z_iden_plot)=find_feature_dist(feature_z,feature_u,z_dummy',u_linear[i,:])
                # #     (mpcCoeff_dummy.c_Vx,mpcCoeff_dummy.c_Vy,mpcCoeff_dummy.c_Psi)=coeff_iden_dist(iden_z,iden_u)
                # #     z_dummy=car_sim_iden_tv(z_dummy,u_linear[i,:],0.1,mpcCoeff_dummy,modelParams,track)
                # #     z_linear[i+1,:]=s6_to_s4(z_dummy')
                # # end

                # # TI SYS_ID
                # (iden_z,iden_u,z_iden_plot)=find_feature_dist(feature_z,feature_u,z_dummy',u_linear[1,:])
                # (mpcCoeff_dummy.c_Vx,mpcCoeff_dummy.c_Vy,mpcCoeff_dummy.c_Psi)=coeff_iden_dist(iden_z,iden_u)
                
                # # tic()
                # for i=1:size(z_linear,1)-1
                # #     # # LOCAL GP
                # #     # GP_e_vy      = regre(z_dummy,u_linear[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                # #     # GP_e_psi_dot = regre(z_dummy,u_linear[i,:],feature_GP_vy_e,feature_GP_z,feature_GP_u)
                    
                # #     # FULL GP
                # #     GP_e_vy      = GP_full(z_dummy,u_linear[i,:],GP_feature,GP_e_vy_prepare)
                # #     GP_e_psi_dot = GP_full(z_dummy,u_linear[i,:],GP_feature,GP_e_psi_dot_prepare)
                    
                # #     # THRESHOLDING
                # #     GP_e_vy      = min(0.025,GP_e_vy)
                # #     GP_e_vy      = max(-0.025,GP_e_vy)
                # #     GP_e_psi_dot = min(0.1,GP_e_psi_dot)
                # #     GP_e_psi_dot = max(-0.1,GP_e_psi_dot)

                #     z_dummy=car_sim_iden_tv(z_dummy,u_linear[i,:],0.1,mpcCoeff_dummy,modelParams,track)
                # #     println("GP_e_vy",GP_e_vy)
                # #     z_dummy[5] += GP_e_vy[1]
                # #     z_dummy[6] += GP_e_psi_dot[1]
                #     z_linear[i+1,:]=s6_to_s4(z_dummy')
                # end   
                # # t = toc();
                # # println("GP time is", t)             

                # # Safe set selection
                # selectedStates=find_SS(oldSS,selectedStates,z_prev,lapStatus,modelParams,mpcParams_4s,track)
                # # selectedStates_kin=deepcopy(selectedStates)
                # selectedStates.selStates=s6_to_s4(selectedStates.selStates)
                
                # # t = toc();
                # # println("elapse prepare time: $t")

                # tic();
                # (mpcSol.z,mpcSol.u,sol_status)=solveMpcProblem_convhull_kin_linear(mdl_kin_lin,mpcParams_4s,modelParams,lapStatus,z_linear,u_linear,z_prev,u_prev,selectedStates,track)
                # t = toc();
                # println("elapse MPC time: $t")

                sol_status_dummy = "$sol_status"
                if sol_status_dummy[1] != 'O'
                    mpcSol.u=copy(u_prev)
                    (mpcSol.z,~,~)=car_pre_dyn(z_curr,mpcSol.u,track,modelParams,6)
                end
            end
            
            # WRITE DATA TO BE SAVED AND UPDATE mpcSol
            saveData(mpcSol,lapStatus,solHistory,oldSS,mpcCoeff)
            # CALCULATE DATA TO BE PLOTTED
            mpcSolPub(mpcSol_to_pub,track,modelParams,mpcSol,selectedStates)

            cmd.motor   = convert(Float32,mpcSol.a_x)
            cmd.servo   = convert(Float32,mpcSol.d_f)
            z_prev      = copy(mpcSol.z)
            u_prev      = copy(mpcSol.u)
            lapStatus.currentIt += 1
        else
            println("No estimation data received!")
        end
        rossleep(loop_rate)
    end # END OF THE WHILE LOOP
    # DATA SAVING
    run_id      = get_param("run_id")
    log_path = "$(homedir())/simulations/LMPC-$(run_id[1:4]).jld"
    if isfile(log_path)
        log_path = "$(homedir())/simulations/LMPC-$(run_id[1:4])-2.jld"
        warn("Warning: File already exists.")
    end
    save(log_path,"log_cvx",log_cvx,"log_cvy",log_cvy,"log_cpsi",log_cpsi,"oldSS",oldSS,"solHistory",solHistory,"track_data",track_data)
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
