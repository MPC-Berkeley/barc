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

# This function is called whenever a new state estimate is received.
# It saves this estimate in oldTraj and uses it in the MPC formulation (see in main)
function SE_callback(msg::pos_info,acc_f::Array{Float64},lapStatus::LapStatus,posInfo::PosInfo,mpcSol::MpcSol,oldTraj::OldTrajectory,z_est::Array{Float64,1},x_est::Array{Float64,1})         # update current position and track data
    # update mpc initial condition
    # println("hahaha")
    z_est[:]                  = [msg.v_x,msg.v_y,msg.psiDot,msg.epsi,msg.ey,msg.s,acc_f[1]] # the last variable is filtered acceleration
    x_est[:]                  = [msg.x,msg.y,msg.psi,msg.v]
    
    # check if lap needs to be switched
    if z_est[6] <= lapStatus.s_lapTrigger && lapStatus.switchLap
        # oldTraj.idx_end[lapStatus.currentLap] = oldTraj.count[lapStatus.currentLap]
        # oldTraj.oldCost[lapStatus.currentLap] = oldTraj.idx_end[lapStatus.currentLap] - oldTraj.idx_start[lapStatus.currentLap]
        lapStatus.nextLap = true
        lapStatus.switchLap = false
    elseif z_est[6] > lapStatus.s_lapTrigger
        lapStatus.switchLap = true
    end
end

function ST_callback(msg::pos_info,z_true::Array{Float64,1})
    z_true[:] = [msg.v_x,msg.v_y,msg.psiDot]
end

# This is the main function, it is called when the node is started.
function main()
    println("Starting LMPC node.")

    # PARAMETER INITILIZATION
    buffersize       = 5000       # size of oldTraj buffers

    # RACING TRACK DATA
    # track_data=[80 0;
    #             120 -pi/2;
    #             80 0;
    #             220 -pi*0.85;
    #             105 pi/15;
    #             300  pi*1.15;
    #             240  -pi*0.865;
    #             100 0;
    #             120 -pi/2;
    #             153 0;
    #             120 -pi/2;
    #             211 0]
    # EXPERIEMENT TRACK DATA
    # num = 100
    # track_data=[80 0;
    #             num -pi/2;
    #             80+47 0;
    #             num -pi/2;
    #             50 0;
    #             num -pi/2;
    #             4 0;
    #             num pi/2;
    #             30 0;
    #             num -pi/2;
    #             4 0;
    #             num -pi/2;
    #             71+48 0]
    # Basic experiment track
    track_data = [60 0;
                  80 -pi/2;
                  20 0;
                  80 -pi/2;
                  40 pi/10;
                  60 -pi/5;
                  40 pi/10;
                  80 -pi/2;
                  20 0;
                  80 -pi/2;
                  75 0]
    # FEATURE TRACK DATA
    # v = 2.5    
    # max_a=7.6;
    # R=v^2/max_a
    # max_c=1/R
    # angle=(pi+pi/2)-0.105
    # R_kin = 0.8
    # num_kin = Int(round(angle/ ( 0.03/R_kin ) * 2))
    # num = max(Int(round(angle/ ( 0.03/R ) * 2)),num_kin)
    # # num*=2
    # track_data=[num -angle;
    #             num  angle]
    track            = Track(track_data)
    oldTraj          = OldTrajectory()
    posInfo          = PosInfo();  posInfo.s_target=track.s;
    mpcCoeff         = MpcCoeff()
    mpcCoeff_dummy   = MpcCoeff()
    lapStatus        = LapStatus(1,1,false,false,0.3)
    mpcSol           = MpcSol()
    modelParams      = ModelParams()
    mpcParams_pF     = MpcParams()
    mpcParams        = MpcParams()  # a dummy parameter in feature data collecting
    mpcParams_4s     = MpcParams()  # For kin_lin LMPC
    selectedStates   = SelectedStates()
    oldSS            = SafeSetData()
    z_est            = zeros(7)          # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
    z_true           = zeros(3)          # (xDot, yDot, psiDot)
    x_est            = zeros(4)          # (x, y, psi, v)
    cmd              = ECU()             # CONTROL SIGNAL MESSAGE INITIALIZATION
    mpcSol_to_pub    = mpc_solution()    # MPC SOLUTION PUBLISHING MESSAGE INITIALIZATION

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

    # DATA LOGGING VARIABLE INITIALIZATION
    num_lap             = 32 # TOTAL NUMBER OF LAPS TO RUN, THE FIRST FEW LAPS DATA MIGHT BE EMEPTY FOR SYS_ID RELATED DATA SAVING
    InitializeParameters(mpcParams,mpcParams_4s,mpcParams_pF,modelParams,posInfo,oldTraj,mpcCoeff,mpcCoeff_dummy,lapStatus,buffersize,selectedStates,oldSS)
    # THOSE INITIALIZATIONS ARE DONE HERE BECAUSE WE NEED INFORMATION FROM THE PARAMETER INITIALIZATION, LIKE PREDICTION HORIZON
    log_cvx             = zeros(buffersize,mpcParams.N,3,num_lap)
    log_cvy             = zeros(buffersize,mpcParams.N,4,num_lap)
    log_cpsi            = zeros(buffersize,mpcParams.N,3,num_lap)
    log_status          = Array(Symbol,buffersize,num_lap)
    solHistory          = SolHistory(500,mpcParams.N,6,num_lap)
    mpcSol.df_his       = zeros(mpcParams.delay_df) # DELAT COMES FROM TWO PARTS, ONLY THE SYSTEM DELAY NEEDS TO BE CONSIDERED

    # MODEL INITIALIZATION
    mdl_pF           = MpcModel_pF(mpcParams_pF,modelParams)
    mdl_convhull     = MpcModel_convhull_dyn_iden(mpcParams,modelParams)
    mdl_kin_lin      = MpcModel_convhull_kin_linear(mpcParams_4s,modelParams)

    # NODE INITIALIZATION
    init_node("mpc_traj")
    loop_rate   = Rate(1/modelParams.dt)
    pub         = Publisher("ecu", ECU, queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
    mpcSol_pub  = Publisher("mpc_solution", mpc_solution, queue_size=1)::RobotOS.Publisher{barc.msg.mpc_solution}
    # The subscriber passes arguments (coeffCurvature and z_est) which are updated by the callback function:
    acc_f       = [0.0]
    s1          = Subscriber("pos_info", pos_info, SE_callback, (acc_f,lapStatus,posInfo,mpcSol,oldTraj,z_est,x_est),queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
    s2          = Subscriber("real_val", pos_info, ST_callback, (z_true,), queue_size=1)::RobotOS.Subscriber{barc.msg.pos_info}
    # Note: Choose queue size long enough so that no pos_info packets get lost! They are important for system ID!
    # HOWEVER, THE NOTE ABOVE IS NOT NEEEDED FOR PRE-SELECTED DATASET
    run_id      = get_param("run_id")
    println("Finished LMPC NODE initialization.")
    
    # THE SIZE OF STATE START WITH 4 SINCE IT ALWAYS START WITH PATH FOLLOWING AND WHEN SWITCHED TO LMPC, mpcSol with be rewritten
    mpcSol.z    = zeros(mpcParams.N+1,4)
    mpcSol.u    = hcat(0.5*ones(mpcParams.N),zeros(mpcParams.N))
    mpcSol.z[1,4] = 1.0 # give the vehcile some initial speed to simulate forward
    for i in 2:mpcParams.N+1
        mpcSol.z[i,:]=car_sim_kin(mpcSol.z[i-1,:],mpcSol.u[i-1,:],track,modelParams)
    end
    mpcSol.a_x  = 0
    mpcSol.d_f  = 0
    z_prev      = mpcSol.z
    u_prev      = mpcSol.u

    # Just for LMPC quick syntax dsebugging
    data = load("$(homedir())/simulations/oldSS.jld")
    oldSS = data["oldSS"]

    # FUNCTION DUMMY CALLS: this is important to call all the functions that will be used before for initial compiling
    oldSS_dummy = data["oldSS"]
    lapStatus_dummy = LapStatus(3,1,false,false,0.3)
    selectedStates_dummy=find_SS(oldSS_dummy,selectedStates,z_prev,lapStatus_dummy,modelParams,mpcParams,track)
    z = rand(1,6); u = rand(1,2)
    (iden_z,iden_u)=find_feature_dist(feature_z,feature_u,z,u)
    (c_Vx,c_Vy,c_Psi)=coeff_iden_dist(iden_z,iden_u)
    (~,~,~)=solveMpcProblem_convhull_dyn_iden(mdl_convhull,mpcParams,mpcSol,mpcCoeff,lapStatus_dummy,rand(6),rand(mpcParams.N+1,6),rand(mpcParams.N,2),selectedStates_dummy,track)
    selectedStates_dummy.selStates=s6_to_s4(selectedStates_dummy.selStates)
    (~,~,~)=solveMpcProblem_convhull_kin_linear(mdl_kin_lin,mpcParams_4s,modelParams,lapStatus,rand(mpcParams.N+1,4),rand(mpcParams.N,2),rand(mpcParams.N+1,6),rand(mpcParams.N,2),selectedStates_dummy,track)
    (~,~,~)=car_pre_dyn(rand(1,6)+1,rand(mpcParams.N,2),track,modelParams,6)
    (~,~,~)=find_SS_dist(solHistory,rand(1,6),rand(1,2),lapStatus)

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
                # SAFE SET COST UPDATE
                oldSS.oldCost[lapStatus.currentLap]     = lapStatus.currentIt-1
                solHistory.cost[lapStatus.currentLap]   = lapStatus.currentIt-1
                oldSS.cost2target[:,lapStatus.currentLap] = lapStatus.currentIt - oldSS.cost2target[:,lapStatus.currentLap]
                lapStatus.nextLap = false
                setvalue(mdl_pF.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,1]-posInfo.s_target)
                setvalue(mdl_pF.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,1]-posInfo.s_target)
                setvalue(mdl_convhull.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,1]-posInfo.s_target)
                setvalue(mdl_convhull.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,1]-posInfo.s_target)
                z_prev[:,1] -= posInfo.s_target

                lapStatus.currentLap += 1
                lapStatus.currentIt = 1
            end

            # OPTIMIZATION
            println("Current Lap: ", lapStatus.currentLap, ", It: ", lapStatus.currentIt, " v: $(z_est[1])")
            if lapStatus.currentLap<=selectedStates.Nl+1 # 1 IS FOR THE FIRST WARM UP LAPS
                # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
                z_curr = [z_est[6],z_est[5],z_est[4],sqrt(z_est[1]^2+z_est[2]^2)]
                (z_sol,u_sol,sol_status)=solveMpcProblem_pathFollow(mdl_pF,mpcParams_pF,modelParams,mpcSol,z_curr,z_prev,u_prev,track)
                
                mpcSol.z = z_sol
                mpcSol.u = u_sol
                mpcSol.a_x = u_sol[1+mpcParams_pF.delay_a,1]
                mpcSol.d_f = u_sol[1+mpcParams_pF.delay_df,2]
                if length(mpcSol.df_his)!=1
                    # INPUT DELAY HISTORY UPDATE
                    mpcSol.df_his[1:end-1] = mpcSol.df_his[2:end]
                    mpcSol.df_his[end] = u_sol[1+mpcParams_pF.delay_df,2]
                end
                z_prev = z_sol
                u_prev = u_sol
                println("LMPC solver status is = $sol_status")
            else
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

                t = toc();
                println("Elapsed preparation time: $t")
                tic()

                (z_sol,u_sol,sol_status)=solveMpcProblem_convhull_dyn_iden(mdl_convhull,mpcParams,mpcSol,mpcCoeff,lapStatus,z_curr,z_prev,u_prev,selectedStates,track)

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
                # (z_sol,u_sol,sol_status)=solveMpcProblem_convhull_kin_linear(mdl_kin_lin,mpcParams_4s,modelParams,lapStatus,z_linear,u_linear,z_prev,u_prev,selectedStates,track)
                # t = toc();
                # println("elapse MPC time: $t")

                ###############################################################
                ###################### DATA WRITING ###########################
                ###############################################################
                sol_status_dummy = "$sol_status"
                if sol_status_dummy[1] != 'O'
                    u_sol=copy(u_prev)
                    (z_sol,~,~)=car_pre_dyn(z_curr,u_sol,track,modelParams,6)
                end
                n_state = size(z_sol,2)
                # mpcSol.z[:,1:n_state] = z_sol
                mpcSol.z = z_sol
                mpcSol.u = u_sol
                mpcSol.a_x = u_sol[1+mpcParams.delay_a,1] 
                mpcSol.d_f = u_sol[1+mpcParams.delay_df,2]
                if length(mpcSol.df_his)!=1
                    # INPUT DELAY HISTORY UPDATE
                    mpcSol.df_his[1:end-1] = mpcSol.df_his[2:end]
                    mpcSol.df_his[end] = u_sol[1+mpcParams.delay_df,2]
                end
                z_prev = copy(z_sol)
                u_prev = copy(u_sol)
                # println("LMPC solver status is = $sol_status")
                solHistory.z[lapStatus.currentIt,lapStatus.currentLap,:,1:n_state]=z_sol
                # solHistory.z[lapStatus.currentIt,lapStatus.currentLap,1,4:6]=z_true # THIS LINE IS NEEDED WHEN KIN_LIN WITH SYS_ID MODEL IS USED
                solHistory.u[lapStatus.currentIt,lapStatus.currentLap,:,:]=u_sol
                (z_fore,~,~) = car_pre_dyn(z_curr,u_sol,track,modelParams,6)
                (z_fore_x,z_fore_y) = trackFrame_to_xyFrame(z_fore,track)
                mpcSol_to_pub.z_fore_x = z_fore_x
                mpcSol_to_pub.z_fore_y = z_fore_y
            end
            cmd.motor = convert(Float32,mpcSol.a_x)
            cmd.servo = convert(Float32,mpcSol.d_f)
            # cmd.header.stamp = get_rostime()
            # publish(pub, cmd)


            # VISUALIZATION COORDINATE CALCULATION FOR view_trajectory.jl NODE
            (z_x,z_y) = trackFrame_to_xyFrame(z_sol,track)
            mpcSol_to_pub.z_x = z_x
            mpcSol_to_pub.z_y = z_y
            (SS_x,SS_y) = trackFrame_to_xyFrame(selectedStates.selStates,track)
            mpcSol_to_pub.SS_x = SS_x
            mpcSol_to_pub.SS_y = SS_y
            mpcSol_to_pub.z_vx = z_sol[:,4]
            mpcSol_to_pub.SS_vx = selectedStates.selStates[:,4]
            mpcSol_to_pub.z_s = z_sol[:,1]
            mpcSol_to_pub.SS_s = selectedStates.selStates[:,1]
            
            # mpcSol_to_pub.header.stamp = get_rostime()     
            # publish(mpcSol_pub, mpcSol_to_pub)
            
            # DATA WRITING AND COUNTER UPDATE
            log_cvx[lapStatus.currentIt,:,:,lapStatus.currentLap]         = mpcCoeff.c_Vx       
            log_cvy[lapStatus.currentIt,:,:,lapStatus.currentLap]         = mpcCoeff.c_Vy       
            log_cpsi[lapStatus.currentIt,:,:,lapStatus.currentLap]        = mpcCoeff.c_Psi
            log_status[lapStatus.currentIt,lapStatus.currentLap]          = mpcSol.solverStatus

            # solHistory.z[lapStatus.currentIt,lapStatus.currentLap,1,:]=[z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
            # solHistory.u[lapStatus.currentIt,lapStatus.currentLap,:,:]=u_sol

            # SAFESET DATA SAVING BASED ON CONTROLLER'S FREQUENCY
            oldSS.oldSS[lapStatus.currentIt,:,lapStatus.currentLap]=[z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
            oldSS.cost2target[lapStatus.currentIt,lapStatus.currentLap]=lapStatus.currentIt
            lapStatus.currentIt += 1
        else
            println("No estimation data received!")
        end
        rossleep(loop_rate)
    end # END OF THE WHILE LOOP
    # DATA SAVING
    log_path = "$(homedir())/simulations/LMPC-$(run_id[1:4]).jld"
    if isfile(log_path)
        log_path = "$(homedir())/simulations/LMPC-$(run_id[1:4])-2.jld"
        warn("Warning: File already exists.")
    end
    save(log_path,"log_cvx",log_cvx,"log_cvy",log_cvy,"log_cpsi",log_cpsi,"oldSS",oldSS,"solHistory",solHistory)
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
