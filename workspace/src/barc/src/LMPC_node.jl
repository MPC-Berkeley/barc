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
    z_est[:]                  = [msg.v_x,msg.v_y,msg.psiDot,msg.epsi,msg.ey,msg.s,acc_f[1]] # the last variable is filtered acceleration
    x_est[:]                  = [msg.x,msg.y,msg.psi,msg.v]

    # check if lap needs to be switched
    if z_est[6] <= lapStatus.s_lapTrigger && lapStatus.switchLap
        oldTraj.idx_end[lapStatus.currentLap] = oldTraj.count[lapStatus.currentLap]
        oldTraj.oldCost[lapStatus.currentLap] = oldTraj.idx_end[lapStatus.currentLap] - oldTraj.idx_start[lapStatus.currentLap]
        lapStatus.nextLap = true
        lapStatus.switchLap = false
    elseif z_est[6] > lapStatus.s_lapTrigger
        lapStatus.switchLap = true
    end

    # save current state in oldTraj
    oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap],:,lapStatus.currentLap] = z_est
    oldTraj.oldInput[oldTraj.count[lapStatus.currentLap],:,lapStatus.currentLap] = [msg.u_a,msg.u_df]
    oldTraj.oldTimes[oldTraj.count[lapStatus.currentLap],lapStatus.currentLap] = to_sec(msg.header.stamp)
    oldTraj.count[lapStatus.currentLap] += 1

    # if necessary: append to end of previous lap
    if lapStatus.currentLap > 1 && z_est[6] < 18.0
        oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap-1],:,lapStatus.currentLap-1] = z_est
        oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap-1],6,lapStatus.currentLap-1] += posInfo.s_target
        oldTraj.oldInput[oldTraj.count[lapStatus.currentLap-1],:,lapStatus.currentLap-1] = [msg.u_a,msg.u_df]
        #oldTraj.oldInput[oldTraj.count[lapStatus.currentLap-1],:,lapStatus.currentLap-1] += 0.5*([msg.u_a msg.u_df]-oldTraj.oldInput[oldTraj.count[lapStatus.currentLap-1]-1,:,lapStatus.currentLap-1])
        oldTraj.oldTimes[oldTraj.count[lapStatus.currentLap-1],lapStatus.currentLap-1] = to_sec(msg.header.stamp)
        oldTraj.count[lapStatus.currentLap-1] += 1
    end

    #if necessary: append to beginning of next lap
    if z_est[6] > posInfo.s_target - 18.0
        oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap+1],:,lapStatus.currentLap+1] = z_est
        oldTraj.oldTraj[oldTraj.count[lapStatus.currentLap+1],6,lapStatus.currentLap+1] -= posInfo.s_target
        oldTraj.oldInput[oldTraj.count[lapStatus.currentLap+1],:,lapStatus.currentLap+1] = [msg.u_a,msg.u_df]
        #oldTraj.oldInput[oldTraj.count[lapStatus.currentLap+1],:,lapStatus.currentLap+1] += 0.5*([msg.u_a msg.u_df]-oldTraj.oldInput[oldTraj.count[lapStatus.currentLap+1]-1,:,lapStatus.currentLap+1])
        oldTraj.oldTimes[oldTraj.count[lapStatus.currentLap+1],lapStatus.currentLap+1] = to_sec(msg.header.stamp)
        oldTraj.count[lapStatus.currentLap+1] += 1
        oldTraj.idx_start[lapStatus.currentLap+1] = oldTraj.count[lapStatus.currentLap+1]
    end
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
    num = 60
    track_data=[80 0;
                num -pi/2;
                80 0;
                num -pi/2;
                50 0;
                num -pi/2;
                4 0;
                num pi/2;
                30 0;
                num -pi/2;
                4 0;
                num -pi/2;
                71 0]
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
    lapStatus        = LapStatus(1,1,false,false,0.3)
    mpcSol           = MpcSol()
    modelParams      = ModelParams()
    mpcParams_pF     = MpcParams()
    mpcParams        = MpcParams()  # a dummy parameter in feature data collecting
    selectedStates   = SelectedStates()
    oldSS            = SafeSetData()
    z_est            = zeros(7)          # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
    x_est            = zeros(4)          # (x, y, psi, v)
    cmd              = ECU()             # CONTROL SIGNAL MESSAGE INITIALIZATION
    mpcSol_to_pub    = mpc_solution()    # MPC SOLUTION PUBLISHING MESSAGE INITIALIZATION
    solHistory = SolHistory(500,10,6,32)

    # FEATURE DATA READING
    data = load("$(homedir())/simulations/Feature_Data/FeatureDataCollecting.jld")
    
    feature_z = data["feature_z"]
    feature_u = data["feature_u"]    

    # DATA LOGGING VARIABLE INITIALIZATION
    # selStates_log    = zeros(selectedStates.Nl*selectedStates.Np,6,buffersize,30)
    # statesCost_log   = zeros(selectedStates.Nl*selectedStates.Np,buffersize,30)
    num_lap = 32
    InitializeParameters(mpcParams,mpcParams_pF,modelParams,posInfo,oldTraj,mpcCoeff,lapStatus,buffersize,selectedStates,oldSS)
    log_cvx                     = zeros(buffersize,mpcParams.N,3,num_lap)
    log_cvy                     = zeros(buffersize,mpcParams.N,4,num_lap)
    log_cpsi                    = zeros(buffersize,mpcParams.N,3,num_lap)
    log_status                  = Array(Symbol,buffersize,num_lap)
    k = 1 # counter initialization, k is the counter from the beginning of the experiment

    # MODEL INITIALIZATION
    mdl_pF           = MpcModel_pF(mpcParams_pF,modelParams)
    mdl_convhull     = MpcModel_convhull_dyn_iden(mpcParams,modelParams)
    
    # NODE INITIALIZATION
    init_node("mpc_traj")
    loop_rate = Rate(1/modelParams.dt)
    pub = Publisher("ecu", ECU, queue_size=1)::RobotOS.Publisher{barc.msg.ECU}
    mpcSol_pub = Publisher("mpc_solution", mpc_solution, queue_size=1)::RobotOS.Publisher{barc.msg.mpc_solution}
    # The subscriber passes arguments (coeffCurvature and z_est) which are updated by the callback function:
    acc_f = [0.0]
    s1 = Subscriber("pos_info", pos_info, SE_callback, (acc_f,lapStatus,posInfo,mpcSol,oldTraj,z_est,x_est),queue_size=50)::RobotOS.Subscriber{barc.msg.pos_info}
    # Note: Choose queue size long enough so that no pos_info packets get lost! They are important for system ID!
    run_id = get_param("run_id")
    println("Finished initialization.")

    # posInfo.s_target        = 19.11 #17.91 #19.14#17.94#17.76#24.0
    
    mpcSol.z = zeros(11,4)
    mpcSol.u = hcat(0.5*ones(10),zeros(10))
    mpcSol.z[1,4] = 1.0 # give the vehcile some initial speed to simulate forward
    for i in 2:11
        mpcSol.z[i,:]=car_sim_kin(mpcSol.z[i-1,:],mpcSol.u[i-1,:],track,modelParams)
    end
    mpcSol.a_x = 0
    mpcSol.d_f = 0
    z_prev = mpcSol.z
    u_prev = mpcSol.u

    # Just for LMPC quick syntax debugging
    data = load("$(homedir())/simulations/oldSS.jld")
    oldSS = data["oldSS"]

    # FUNCTION DUMMY CALLS: this is important to call all the functions that will be used before for initial compiling
    data = load("$(homedir())/simulations/oldSS.jld")
    oldSS_dummy = data["oldSS"]
    lapStatus_dummy = LapStatus(3,1,false,false,0.3)
    selectedStates_dummy=find_SS(oldSS_dummy,selectedStates,z_prev,lapStatus_dummy,modelParams,mpcParams,track)
    z = rand(1,6); u = rand(1,2)
    (iden_z,iden_u)=find_feature_dist(feature_z,feature_u,z,u)
    (c_Vx,c_Vy,c_Psi)=coeff_iden_dist(iden_z,iden_u)
    (~,~,~)=solveMpcProblem_convhull_dyn_iden(mdl_convhull,mpcParams,mpcCoeff,lapStatus_dummy,rand(6),rand(11,6),rand(10,2),selectedStates_dummy,track)    
    (~,~,~)=car_pre_dyn(rand(1,6)+1,rand(10,2),track,modelParams,6)

    while ! is_shutdown()
        if z_est[6] > 0    
            # CONTROL SIGNAL PUBLISHING
            cmd.header.stamp = get_rostime()
            mpcSol_to_pub.header.stamp = get_rostime()     
            publish(pub, cmd)
            publish(mpcSol_pub, mpcSol_to_pub)

            # LAP SWITCHING
            if lapStatus.nextLap
                println("Finishing one lap at iteration ",lapStatus.currentIt)
                # SAFE SET COST UPDATE
                # log_final_counter[lapStatus.currentLap-1] = k
                oldSS.oldCost[lapStatus.currentLap] = lapStatus.currentIt-1
                # cost2target                           = zeros(buffersize) # array containing the cost to arrive from each point of the old trajectory to the target            
                # for j = 1:buffersize
                #     cost2target[j] = (lapStatus.currentIt-j+1)  
                # end
                oldSS.cost2target[:,lapStatus.currentLap] = lapStatus.currentIt - oldSS.cost2target[:,lapStatus.currentLap]
                lapStatus.nextLap = false
                # if mpcSol.z[1,1]>posInfo.s_target
                    # WARM START SWITCHING
                setvalue(mdl_pF.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,1]-posInfo.s_target)
                setvalue(mdl_pF.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,1]-posInfo.s_target)
                setvalue(mdl_convhull.z_Ol[1:mpcParams.N,1],mpcSol.z[2:mpcParams.N+1,1]-posInfo.s_target)
                setvalue(mdl_convhull.z_Ol[mpcParams.N+1,1],mpcSol.z[mpcParams.N+1,1]-posInfo.s_target)
                z_prev[:,1] -= posInfo.s_target

                # end
                lapStatus.currentLap += 1
                lapStatus.currentIt = 1
            end

            # OPTIMIZATION
            println("Current Lap: ", lapStatus.currentLap, ", It: ", lapStatus.currentIt)
            if lapStatus.currentLap<=3 # FOR QUICK LMPC DEBUGGING
                # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
                z_curr = [z_est[6],z_est[5],z_est[4],sqrt(z_est[1]^2+z_est[2]^2)]
                (z_sol,u_sol,sol_status)=solveMpcProblem_pathFollow(mdl_pF,mpcParams_pF,modelParams,z_curr,z_prev,u_prev,track)
                mpcSol.z = z_sol
                mpcSol.u = u_sol
                mpcSol.a_x = u_sol[2,1]
                mpcSol.d_f = u_sol[2,2]
                z_prev = z_sol
                u_prev = u_sol
                println("LMPC solver status is = $sol_status")
            else
                # save("$(homedir())/simulations/oldSS.jld","oldSS",oldSS)
                # tic()
                # lapStatus.currentLap = 4
                # ESTIMATED STATES PARSING
                # (xDot, yDot, psiDot, ePsi, eY, s, acc_f)
                z_curr = [z_est[6],z_est[5],z_est[4],z_est[1],z_est[2],z_est[3]]
                # println("s:", z_curr[1], " x:", x_est[1], " y:", x_est[2])
                # SAFESET POINT SELECTION
                selectedStates=find_SS(oldSS,selectedStates,z_prev,lapStatus,modelParams,mpcParams,track)

                # Temperal solution: will be improved
                if size(z_prev,2)==4
                    z_prev = hcat(z_prev,zeros(size(z_prev,1),2))
                end

                # # FEATURE POINT SELECTION AND SYS_ID
                # z_to_iden = vcat(z_curr',z_prev[3:end,:])
                # u_to_iden = vcat(u_prev[2:end,:],u_prev[end,:])
                # # TV SYS_ID
                # for i in 1:mpcParams.N
                #     z = z_to_iden[i,:]
                #     u = u_to_iden[i,:]
                #     (iden_z,iden_u)=find_feature_dist(feature_z,feature_u,z,u)
                #     (mpcCoeff.c_Vx[i,:],mpcCoeff.c_Vy[i,:],mpcCoeff.c_Psi[i,:])=coeff_iden_dist(iden_z,iden_u)
                # end
                # println(hcat(z_curr[4:6]',u_prev[2,:]))
                # println(hcat(iden_z[:,:,1],iden_u))

                # TI SYS_ID
                z_to_iden = vcat(z_curr',z_prev[3:end,:])
                u_to_iden = vcat(u_prev[2:end,:],u_prev[end,:])
                z = z_to_iden[1,:]
                u = u_to_iden[1,:]
                (iden_z,iden_u)=find_feature_dist(feature_z,feature_u,z,u)
                println(hcat(z_curr[4:6]',u_prev[2,:]))
                println(hcat(iden_z[:,:,1],iden_u))
                (mpcCoeff.c_Vx[1,:],mpcCoeff.c_Vy[1,:],mpcCoeff.c_Psi[1,:])=coeff_iden_dist(iden_z,iden_u)
                for i in 2:mpcParams.N
                    mpcCoeff.c_Vx[i,:]=mpcCoeff.c_Vx[1,:]
                    mpcCoeff.c_Vy[i,:]=mpcCoeff.c_Vy[1,:]
                    mpcCoeff.c_Psi[i,:]=mpcCoeff.c_Psi[1,:]
                end

                # t = toc()
                # println("elapse time: $t")
                # tic()
                # LMPC CONTROLLER OPTIMIZATION
                # println("mpcCoeff.Vx: $(mpcCoeff.c_Vx)")
                # println("mpcCoeff.Vy: $(mpcCoeff.c_Vy)")
                # println("mpcCoeff.Psi: $(mpcCoeff.c_Psi)")
                # println("z_curr: $z_curr")
                # println("z_prev: $z_prev")
                println("u: $(u_prev[2,:])")
                # println("SelectedState: $selectedStates")
                # println("mpcParams: $mpcParams")
                (z_sol,u_sol,sol_status)=solveMpcProblem_convhull_dyn_iden(mdl_convhull,mpcParams,mpcCoeff,lapStatus,z_curr,z_prev,u_prev,selectedStates,track)
                # In case it is not optimal solution
                # save("$(homedir())/status.jld","sol_status",sol_status)
                # t = toc()
                # println("elapse time: $t")

                sol_status_dummy = "$sol_status"
                if sol_status_dummy[1] != 'O'
                    u_sol=copy(u_prev)
                    (z_sol,~,~)=car_pre_dyn(z_curr,u_sol,track,modelParams,6)
                end
                mpcSol.z = z_sol
                mpcSol.u = u_sol
                mpcSol.a_x = u_sol[2,1] 
                mpcSol.d_f = u_sol[2,2]
                z_prev = z_sol
                u_prev = u_sol
                println("LMPC solver status is = $sol_status")
                solHistory.z[lapStatus.currentIt,lapStatus.currentLap,:,:]=z_sol
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
